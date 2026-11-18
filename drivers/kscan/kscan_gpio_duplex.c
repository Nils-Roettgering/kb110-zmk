/*
 * Copyright (c) 2026 kb110 contributors
 * SPDX-License-Identifier: MIT
 *
 * Two-phase ("duplex") GPIO matrix kscan driver.
 *
 * Some keyboard matrices place two switches on every (row, col) cell to halve
 * the number of MCU pins needed. Each switch has its own diode and the two
 * diodes of a cell point in opposite directions:
 *
 *   - "col2row" switches conduct col -> row and are only detectable while
 *     their column is driven and their row is read,
 *   - "row2col" switches conduct row -> col and are only detectable while
 *     their row is driven and their column is read.
 *
 * Every switch must therefore be read from a single driven pin (driving whole
 * nets at once would merge several switches into one input reading). The scan
 * cycle drives each of the 10 columns (reading the 6 rows) and then each of
 * the 6 rows (reading the 10 columns).
 *
 * Because the anti-parallel diodes make every row<->col connection
 * bidirectional, three pressed switches can form a sneak path that pulls a
 * fourth, unpressed slot high (classic ghosting, which plain single-direction
 * matrices avoid by construction). This driver suppresses those ghosts: a
 * reading is dropped when the other two legs of a 3-switch corner are visible
 * in the same scan cycle, so phantom keys are filtered at the cost of dropping
 * the fourth key of an exact 4-key corner (the standard trade-off of
 * ghosting-limited matrices; any 3 keys are always reported correctly).
 *
 * Power behaviour:
 *   - active (any key down / debouncing): full scan cycles run back to back at
 *     debounce-scan-period-ms,
 *   - idle: one full cycle every poll-period-ms, and additionally the input
 *     pins of an all-drives pattern are armed with level interrupts so typical
 *     key presses wake the scan immediately (the all-drives patterns cannot
 *     ghost and are only used as wake detectors, never for key readings).
 *
 * Event slots: col2row switch of cell (row r, col c) -> event (r, c);
 * row2col switch of cell (row r, col c) -> event (rows_len + r, c). The
 * `skip-cells` property lists logical (row, col) pairs that must never report
 * events (e.g. the duplicate detection of a diode-less encoder push button).
 */

#define DT_DRV_COMPAT zmk_kscan_gpio_duplex

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>

#include <zmk/debounce.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define INST_ROWS_LEN(n) DT_INST_PROP_LEN(n, row_gpios)
#define INST_COLS_LEN(n) DT_INST_PROP_LEN(n, col_gpios)
#define INST_MATRIX_ROWS(n) (2 * INST_ROWS_LEN(n))
#define INST_MATRIX_LEN(n) (INST_MATRIX_ROWS(n) * INST_COLS_LEN(n))

#define INST_DEBOUNCE_PRESS_MS(n) DT_INST_PROP(n, debounce_press_ms)
#define INST_DEBOUNCE_RELEASE_MS(n) DT_INST_PROP(n, debounce_release_ms)

#define KSCAN_ROW_CFG_INIT(idx, inst_idx)                                                          \
    GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst_idx), row_gpios, idx)
#define KSCAN_COL_CFG_INIT(idx, inst_idx)                                                          \
    GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(inst_idx), col_gpios, idx)

/** Maximum elapsed time fed to the debouncer in one step (keeps the integrator sane). */
#define MAX_DEBOUNCE_ELAPSED_MS 500

/** Which all-drives wake pattern is armed while idle. */
enum idle_wake_pattern {
    WAKE_COLS_DRIVEN = 0, /* all cols driven, rows read (col2row presses interrupt) */
    WAKE_ROWS_DRIVEN = 1, /* all rows driven, cols read (row2col presses interrupt) */
};

struct kscan_duplex_irq_data {
    const struct device *dev;
    struct gpio_callback callback;
};

struct kscan_duplex_data {
    const struct device *dev;
    kscan_callback_t callback;
    struct k_work_delayable work;
    struct kscan_duplex_irq_data *irqs;
    /** Timestamp of the current or scheduled scan. */
    int64_t scan_time;
    /** Timestamp of the previous full scan cycle (for debounce elapsed). */
    int64_t last_cycle_time;
    /** Wake pattern to arm next while idle. */
    enum idle_wake_pattern wake_pattern;
    /** Raw row readings per col-drive pass: rows_read[col][row]. */
    uint8_t *rows_read;
    /** Raw col readings per row-drive pass: cols_read[row][col]. */
    uint8_t *cols_read;
    /** Debounce state, indexed like the events: (col * matrix_rows) + logical_row. */
    struct zmk_debounce_state *matrix_state;
};

struct kscan_duplex_config {
    size_t rows_len;
    size_t cols_len;
    const struct gpio_dt_spec *rows;
    const struct gpio_dt_spec *cols;
    struct zmk_debounce_config debounce_config;
    int32_t debounce_scan_period_ms;
    int32_t poll_period_ms;
    /** Logical (row, col) pairs which must never emit events. */
    const uint8_t *skip_cells;
    size_t skip_len;
};

static bool is_skipped(const struct kscan_duplex_config *config, const int row, const int col) {
    for (size_t i = 0; i + 1 < config->skip_len; i += 2) {
        if (config->skip_cells[i] == row && config->skip_cells[i + 1] == col) {
            return true;
        }
    }
    return false;
}

/** Index into the debounce state array from a logical row and column. */
static int state_index_rc(const struct kscan_duplex_config *config, const int row, const int col) {
    __ASSERT(row >= 0 && (size_t)row < 2 * config->rows_len, "Invalid row %i", row);
    __ASSERT(col >= 0 && (size_t)col < config->cols_len, "Invalid column %i", col);

    return (col * 2 * (int)config->rows_len) + row;
}

static int kscan_duplex_set_pins_as_inputs(const struct gpio_dt_spec *pins, size_t len) {
    for (size_t i = 0; i < len; i++) {
        int err = gpio_pin_configure_dt(&pins[i], GPIO_INPUT);
        if (err) {
            LOG_ERR("Unable to configure pin %u on %s as input", pins[i].pin, pins[i].port->name);
            return err;
        }
    }
    return 0;
}

static int kscan_duplex_drive_pins(const struct gpio_dt_spec *pins, size_t len) {
    for (size_t i = 0; i < len; i++) {
        int err = gpio_pin_configure_dt(&pins[i], GPIO_OUTPUT);
        if (err) {
            LOG_ERR("Unable to configure pin %u on %s as output", pins[i].pin,
                    pins[i].port->name);
            return err;
        }
        err = gpio_pin_set_dt(&pins[i], 1);
        if (err) {
            LOG_ERR("Failed to drive pin %u on %s active", pins[i].pin, pins[i].port->name);
            return err;
        }
    }
    return 0;
}

/** Stop driving a set of pins and release them back to (pulled) inputs. */
static int kscan_duplex_release_pins(const struct gpio_dt_spec *pins, size_t len) {
    for (size_t i = 0; i < len; i++) {
        int err = gpio_pin_set_dt(&pins[i], 0);
        if (err) {
            return err;
        }
        err = gpio_pin_configure_dt(&pins[i], GPIO_INPUT);
        if (err) {
            return err;
        }
    }
    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)
static int kscan_duplex_disconnect_pins(const struct gpio_dt_spec *pins, size_t len) {
    for (size_t i = 0; i < len; i++) {
        int err = gpio_pin_configure_dt(&pins[i], GPIO_DISCONNECTED);
        if (err) {
            return err;
        }
    }
    return 0;
}
#endif // IS_ENABLED(CONFIG_PM_DEVICE)

/** Read one input set into dst while a single output pin is driven. */
static int kscan_duplex_read_inputs(const struct gpio_dt_spec *inputs, size_t len, uint8_t *dst) {
    for (size_t i = 0; i < len; i++) {
        const int active = gpio_pin_get_dt(&inputs[i]);
        if (active < 0) {
            LOG_ERR("Failed to read pin %u on %s: %i", inputs[i].pin, inputs[i].port->name, active);
            return active;
        }
        dst[i] = (uint8_t)(active ? 1u : 0u);
    }
    return 0;
}

/**
 * One full scan cycle: drive each column and read the rows, then drive each
 * row and read the columns.
 */
static int kscan_duplex_scan_cycle(const struct device *dev) {
    struct kscan_duplex_data *data = dev->data;
    const struct kscan_duplex_config *config = dev->config;

    for (size_t c = 0; c < config->cols_len; c++) {
        int err = kscan_duplex_drive_pins(&config->cols[c], 1);
        if (err) {
            return err;
        }
        err = kscan_duplex_read_inputs(config->rows, config->rows_len,
                                       &data->rows_read[c * config->rows_len]);
        if (err) {
            return err;
        }
        err = kscan_duplex_release_pins(&config->cols[c], 1);
        if (err) {
            return err;
        }
    }

    for (size_t r = 0; r < config->rows_len; r++) {
        int err = kscan_duplex_drive_pins(&config->rows[r], 1);
        if (err) {
            return err;
        }
        err = kscan_duplex_read_inputs(config->cols, config->cols_len,
                                       &data->cols_read[r * config->cols_len]);
        if (err) {
            return err;
        }
        err = kscan_duplex_release_pins(&config->rows[r], 1);
        if (err) {
            return err;
        }
    }

    return 0;
}

/**
 * Ghost suppression: a col2row reading of cell (r, c) is only trustworthy if no
 * 3-switch sneak path c -> r' -> c' -> r exists through other pressed switches.
 * All three legs of such a path are visible in the raw readings of this cycle.
 */
static bool col_slot_is_ghost(const struct kscan_duplex_config *config,
                              const struct kscan_duplex_data *data, size_t r, size_t c) {
    for (size_t r2 = 0; r2 < config->rows_len; r2++) {
        if (r2 == r || !data->rows_read[c * config->rows_len + r2]) {
            continue;
        }
        for (size_t c2 = 0; c2 < config->cols_len; c2++) {
            if (c2 == c || !data->cols_read[r2 * config->cols_len + c2]) {
                continue;
            }
            if (data->rows_read[c2 * config->rows_len + r]) {
                return true;
            }
        }
    }
    return false;
}

/** Mirror image of col_slot_is_ghost() for row2col slots. */
static bool row_slot_is_ghost(const struct kscan_duplex_config *config,
                              const struct kscan_duplex_data *data, size_t r, size_t c) {
    for (size_t c2 = 0; c2 < config->cols_len; c2++) {
        if (c2 == c || !data->cols_read[r * config->cols_len + c2]) {
            continue;
        }
        for (size_t r2 = 0; r2 < config->rows_len; r2++) {
            if (r2 == r || !data->rows_read[c2 * config->rows_len + r2]) {
                continue;
            }
            if (data->cols_read[r2 * config->cols_len + c]) {
                return true;
            }
        }
    }
    return false;
}

/** Feed one cycle of readings into the debouncer and emit change events. */
static void kscan_duplex_process_cycle(const struct device *dev, int elapsed_ms,
                                       bool *any_active) {
    struct kscan_duplex_data *data = dev->data;
    const struct kscan_duplex_config *config = dev->config;

    for (size_t c = 0; c < config->cols_len; c++) {
        for (size_t r = 0; r < config->rows_len; r++) {
            bool active = data->rows_read[c * config->rows_len + r] &&
                          !col_slot_is_ghost(config, data, r, c);

            zmk_debounce_update(
                &data->matrix_state[state_index_rc(config, (int)r, (int)c)], active, elapsed_ms,
                &config->debounce_config);
        }
    }

    for (size_t c = 0; c < config->cols_len; c++) {
        for (size_t r = 0; r < config->rows_len; r++) {
            const size_t logical_row = r + config->rows_len;
            bool active = data->cols_read[r * config->cols_len + c] &&
                          !row_slot_is_ghost(config, data, r, c);

            zmk_debounce_update(&data->matrix_state[state_index_rc(config, (int)logical_row,
                                                                  (int)c)],
                                active, elapsed_ms, &config->debounce_config);
        }
    }

    for (size_t c = 0; c < config->cols_len; c++) {
        for (size_t lr = 0; lr < 2 * config->rows_len; lr++) {
            if (is_skipped(config, (int)lr, (int)c)) {
                continue;
            }

            struct zmk_debounce_state *state =
                &data->matrix_state[state_index_rc(config, (int)lr, (int)c)];

            if (zmk_debounce_get_changed(state)) {
                const bool pressed = zmk_debounce_is_pressed(state);

                LOG_DBG("Sending event at %zu,%zu state %s", lr, c, pressed ? "on" : "off");
                data->callback(dev, (uint32_t)lr, (uint32_t)c, pressed);
            }

            *any_active = *any_active || zmk_debounce_is_active(state);
        }
    }
}

static int kscan_duplex_interrupt_configure(const struct gpio_dt_spec *pins, size_t len,
                                            const gpio_flags_t flags) {
    for (size_t i = 0; i < len; i++) {
        int err = gpio_pin_interrupt_configure_dt(&pins[i], flags);
        if (err) {
            LOG_ERR("Unable to configure interrupt for pin %u on %s", pins[i].pin,
                    pins[i].port->name);
            return err;
        }
    }
    return 0;
}

/** Arm the level interrupts of the input pins of the given wake pattern. */
static int kscan_duplex_wake_arm(const struct device *dev, const enum idle_wake_pattern pattern) {
    const struct kscan_duplex_config *config = dev->config;
    const struct gpio_dt_spec *reads =
        (pattern == WAKE_COLS_DRIVEN) ? config->rows : config->cols;
    const size_t reads_len = (pattern == WAKE_COLS_DRIVEN) ? config->rows_len : config->cols_len;
    const struct gpio_dt_spec *drives =
        (pattern == WAKE_COLS_DRIVEN) ? config->cols : config->rows;
    const size_t drives_len = (pattern == WAKE_COLS_DRIVEN) ? config->cols_len : config->rows_len;

    int err = kscan_duplex_drive_pins(drives, drives_len);
    if (err) {
        return err;
    }
    err = kscan_duplex_set_pins_as_inputs(reads, reads_len);
    if (err) {
        return err;
    }
    return kscan_duplex_interrupt_configure(reads, reads_len, GPIO_INT_LEVEL_ACTIVE);
}

/** Disarm the wake pattern: stop driving and disable all interrupts. */
static int kscan_duplex_wake_disarm(const struct device *dev) {
    const struct kscan_duplex_config *config = dev->config;
    int err = kscan_duplex_interrupt_configure(config->rows, config->rows_len, GPIO_INT_DISABLE);
    if (err) {
        return err;
    }
    err = kscan_duplex_interrupt_configure(config->cols, config->cols_len, GPIO_INT_DISABLE);
    if (err) {
        return err;
    }
    err = kscan_duplex_release_pins(config->rows, config->rows_len);
    if (err) {
        return err;
    }
    return kscan_duplex_release_pins(config->cols, config->cols_len);
}

static void kscan_duplex_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct kscan_duplex_data *data = CONTAINER_OF(dwork, struct kscan_duplex_data, work);
    const struct device *dev = data->dev;
    const struct kscan_duplex_config *config = dev->config;

    bool any_active = false;
    int err;

    // Never scan while a wake pattern is armed/driving: disarm first (safe to
    // call even when nothing is armed).
    err = kscan_duplex_wake_disarm(dev);
    if (err) {
        LOG_ERR("Failed to disarm wake pattern: %i", err);
        return;
    }

    // A full scan cycle takes well below 1 ms; debounce it against the wall clock.
    const int64_t now = k_uptime_get();
    int elapsed = (int)(now - data->last_cycle_time);
    if (elapsed < 0) {
        elapsed = 0;
    }
    if (elapsed > MAX_DEBOUNCE_ELAPSED_MS) {
        elapsed = MAX_DEBOUNCE_ELAPSED_MS;
    }
    data->last_cycle_time = now;

    err = kscan_duplex_scan_cycle(dev);
    if (err) {
        LOG_ERR("Scan failed: %i", err);
        return;
    }

    kscan_duplex_process_cycle(dev, elapsed, &any_active);

    if (any_active) {
        // At least one key is pressed or undecided: keep scanning quickly.
        data->scan_time += config->debounce_scan_period_ms;
    } else {
        // Idle: arm the alternating wake pattern and poll slowly.
        err = kscan_duplex_wake_arm(dev, data->wake_pattern);
        if (err) {
            LOG_ERR("Failed to arm wake pattern: %i", err);
            return;
        }
        data->wake_pattern = (data->wake_pattern == WAKE_COLS_DRIVEN) ? WAKE_ROWS_DRIVEN
                                                                     : WAKE_COLS_DRIVEN;
        data->scan_time += config->poll_period_ms;
    }

    k_work_reschedule(&data->work, K_TIMEOUT_ABS_MS(data->scan_time));
}

static void kscan_duplex_irq_callback_handler(const struct device *port, struct gpio_callback *cb,
                                              const gpio_port_pins_t pin) {
    struct kscan_duplex_irq_data *irq_data =
        CONTAINER_OF(cb, struct kscan_duplex_irq_data, callback);
    struct kscan_duplex_data *data = irq_data->dev->data;

    // Disable our interrupts and drive lines to avoid re-entry while we scan.
    kscan_duplex_wake_disarm(irq_data->dev);

    data->scan_time = k_uptime_get();

    k_work_reschedule(&data->work, K_NO_WAIT);
}

static int kscan_duplex_configure(const struct device *dev, const kscan_callback_t callback) {
    struct kscan_duplex_data *data = dev->data;

    if (!callback) {
        return -EINVAL;
    }

    data->callback = callback;
    return 0;
}

static int kscan_duplex_enable(const struct device *dev) {
    struct kscan_duplex_data *data = dev->data;

    data->scan_time = k_uptime_get();

    // Start with a full scan, the handler returns to idle polling afterwards.
    return k_work_reschedule(&data->work, K_NO_WAIT);
}

static int kscan_duplex_disable(const struct device *dev) {
    struct kscan_duplex_data *data = dev->data;

    k_work_cancel_delayable(&data->work);

    return kscan_duplex_wake_disarm(dev);
}

static int kscan_duplex_init_pins(const struct device *dev) {
    const struct kscan_duplex_config *config = dev->config;

    for (size_t i = 0; i < config->rows_len; i++) {
        if (!device_is_ready(config->rows[i].port)) {
            LOG_ERR("GPIO is not ready: %s", config->rows[i].port->name);
            return -ENODEV;
        }
    }
    for (size_t i = 0; i < config->cols_len; i++) {
        if (!device_is_ready(config->cols[i].port)) {
            LOG_ERR("GPIO is not ready: %s", config->cols[i].port->name);
            return -ENODEV;
        }
    }

    int err = kscan_duplex_set_pins_as_inputs(config->rows, config->rows_len);
    if (err) {
        return err;
    }
    err = kscan_duplex_set_pins_as_inputs(config->cols, config->cols_len);
    if (err) {
        return err;
    }

    // Register the level interrupt callbacks for every pin. Interrupts are
    // enabled/disabled dynamically by the idle wake pattern.
    struct kscan_duplex_data *data = dev->data;
    for (size_t i = 0; i < config->rows_len; i++) {
        data->irqs[i].dev = dev;
        gpio_init_callback(&data->irqs[i].callback, kscan_duplex_irq_callback_handler,
                           BIT(config->rows[i].pin));
        err = gpio_add_callback(config->rows[i].port, &data->irqs[i].callback);
        if (err) {
            LOG_ERR("Error adding the row callback: %i", err);
            return err;
        }
    }
    for (size_t i = 0; i < config->cols_len; i++) {
        data->irqs[config->rows_len + i].dev = dev;
        gpio_init_callback(&data->irqs[config->rows_len + i].callback,
                           kscan_duplex_irq_callback_handler, BIT(config->cols[i].pin));
        err = gpio_add_callback(config->cols[i].port, &data->irqs[config->rows_len + i].callback);
        if (err) {
            LOG_ERR("Error adding the col callback: %i", err);
            return err;
        }
    }

    return 0;
}

#if IS_ENABLED(CONFIG_PM_DEVICE)

static int kscan_duplex_pm_action(const struct device *dev, enum pm_device_action action) {
    struct kscan_duplex_data *data = dev->data;
    const struct kscan_duplex_config *config = dev->config;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        k_work_cancel_delayable(&data->work);
        kscan_duplex_wake_disarm(dev);
        kscan_duplex_disconnect_pins(config->rows, config->rows_len);
        return kscan_duplex_disconnect_pins(config->cols, config->cols_len);
    case PM_DEVICE_ACTION_RESUME:
        data->scan_time = k_uptime_get();
        kscan_duplex_init_pins(dev);
        return k_work_reschedule(&data->work, K_NO_WAIT);
    default:
        return -ENOTSUP;
    }
}

#endif // IS_ENABLED(CONFIG_PM_DEVICE)

static int kscan_duplex_init(const struct device *dev) {
    struct kscan_duplex_data *data = dev->data;

    data->dev = dev;

    k_work_init_delayable(&data->work, kscan_duplex_work_handler);

#if IS_ENABLED(CONFIG_PM_DEVICE)
    pm_device_init_suspended(dev);

#if IS_ENABLED(CONFIG_PM_DEVICE_RUNTIME)
    pm_device_runtime_enable(dev);
#endif

#else
    return kscan_duplex_init_pins(dev);
#endif

    return 0;
}

static const struct kscan_driver_api kscan_duplex_api = {
    .config = kscan_duplex_configure,
    .enable_callback = kscan_duplex_enable,
    .disable_callback = kscan_duplex_disable,
};

#define KSCAN_DUPLEX_INIT(n)                                                                       \
    BUILD_ASSERT(INST_DEBOUNCE_PRESS_MS(n) <= DEBOUNCE_COUNTER_MAX,                                \
                 "debounce-press-ms is too large");                                                \
    BUILD_ASSERT(INST_DEBOUNCE_RELEASE_MS(n) <= DEBOUNCE_COUNTER_MAX,                              \
                 "debounce-release-ms is too large");                                              \
    BUILD_ASSERT(!DT_INST_NODE_HAS_PROP(n, skip_cells) ||                                          \
                     (DT_INST_PROP_LEN(n, skip_cells) % 2) == 0,                                   \
                 "skip-cells must contain (row, col) pairs");                                      \
                                                                                                   \
    static struct gpio_dt_spec kscan_duplex_rows_##n[] = {                                         \
        LISTIFY(INST_ROWS_LEN(n), KSCAN_ROW_CFG_INIT, (, ), n)};                                   \
    static struct gpio_dt_spec kscan_duplex_cols_##n[] = {                                         \
        LISTIFY(INST_COLS_LEN(n), KSCAN_COL_CFG_INIT, (, ), n)};                                   \
    static struct zmk_debounce_state kscan_duplex_state_##n[INST_MATRIX_LEN(n)];                   \
    static uint8_t kscan_duplex_rows_read_##n[INST_COLS_LEN(n) * INST_ROWS_LEN(n)];                \
    static uint8_t kscan_duplex_cols_read_##n[INST_ROWS_LEN(n) * INST_COLS_LEN(n)];                \
    static struct kscan_duplex_irq_data                                                            \
        kscan_duplex_irqs_##n[INST_ROWS_LEN(n) + INST_COLS_LEN(n)];                                \
    IF_ENABLED(DT_INST_NODE_HAS_PROP(n, skip_cells),                                               \
               (static const uint8_t kscan_duplex_skip_##n[] = DT_INST_PROP(n, skip_cells);))      \
                                                                                                   \
    static struct kscan_duplex_data kscan_duplex_data_##n = {                                      \
        .irqs = kscan_duplex_irqs_##n,                                                             \
        .rows_read = kscan_duplex_rows_read_##n,                                                   \
        .cols_read = kscan_duplex_cols_read_##n,                                                   \
        .matrix_state = kscan_duplex_state_##n,                                                    \
        .wake_pattern = WAKE_COLS_DRIVEN,                                                          \
    };                                                                                             \
                                                                                                   \
    static const struct kscan_duplex_config kscan_duplex_config_##n = {                            \
        .rows_len = ARRAY_SIZE(kscan_duplex_rows_##n),                                             \
        .cols_len = ARRAY_SIZE(kscan_duplex_cols_##n),                                             \
        .rows = kscan_duplex_rows_##n,                                                             \
        .cols = kscan_duplex_cols_##n,                                                             \
        .debounce_config =                                                                         \
            {                                                                                      \
                .debounce_press_ms = INST_DEBOUNCE_PRESS_MS(n),                                    \
                .debounce_release_ms = INST_DEBOUNCE_RELEASE_MS(n),                                \
            },                                                                                     \
        .debounce_scan_period_ms = DT_INST_PROP(n, debounce_scan_period_ms),                       \
        .poll_period_ms = DT_INST_PROP(n, poll_period_ms),                                         \
        .skip_cells = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, skip_cells),                            \
                                  (kscan_duplex_skip_##n), (NULL)),                                \
        .skip_len = COND_CODE_1(DT_INST_NODE_HAS_PROP(n, skip_cells),                              \
                                (DT_INST_PROP_LEN(n, skip_cells)), (0)),                           \
    };                                                                                             \
                                                                                                   \
    PM_DEVICE_DT_INST_DEFINE(n, kscan_duplex_pm_action);                                           \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, &kscan_duplex_init, PM_DEVICE_DT_INST_GET(n), &kscan_duplex_data_##n, \
                          &kscan_duplex_config_##n, POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY,       \
                          &kscan_duplex_api);

DT_INST_FOREACH_STATUS_OKAY(KSCAN_DUPLEX_INIT);
