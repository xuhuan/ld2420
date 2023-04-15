import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_TIMEOUT
from esphome import automation
from esphome.automation import maybe_simple_id

DEPENDENCIES = ["uart"]
CODEOWNERS = ["@GelidusResearch"]
MULTI_CONF = True

ld2420_ns = cg.esphome_ns.namespace("ld2420")
LD2420Component = ld2420_ns.class_("LD2420Component", cg.Component, uart.UARTDevice)
LD2420Restart = ld2420_ns.class_("LD2420Restart", automation.Action)
CONF_LD2420_ID = "ld2420_id"


CONF_DETECTION_GATE_MIN = "detection_gate_minimum"
CONF_DETECTION_GATE_MAX = "detection_gate_maximum"
CONF_PRESENCE_TIME_WINDOW = "presence_time_window"
CONF_G0_MOVE_THRESHOLD = "g0_move_threshold"
CONF_G0_STILL_THRESHOLD = "g0_still_threshold"
CONF_G1_MOVE_THRESHOLD = "g1_move_threshold"
CONF_G1_STILL_THRESHOLD = "g1_still_threshold"
CONF_G2_MOVE_THRESHOLD = "g2_move_threshold"
CONF_G2_STILL_THRESHOLD = "g2_still_threshold"
CONF_G3_MOVE_THRESHOLD = "g3_move_threshold"
CONF_G3_STILL_THRESHOLD = "g3_still_threshold"
CONF_G4_MOVE_THRESHOLD = "g4_move_threshold"
CONF_G4_STILL_THRESHOLD = "g4_still_threshold"
CONF_G5_MOVE_THRESHOLD = "g5_move_threshold"
CONF_G5_STILL_THRESHOLD = "g5_still_threshold"
CONF_G6_MOVE_THRESHOLD = "g6_move_threshold"
CONF_G6_STILL_THRESHOLD = "g6_still_threshold"
CONF_G7_MOVE_THRESHOLD = "g7_move_threshold"
CONF_G7_STILL_THRESHOLD = "g7_still_threshold"
CONF_G8_MOVE_THRESHOLD = "g8_move_threshold"
CONF_G8_STILL_THRESHOLD = "g8_still_threshold"
CONF_G9_MOVE_THRESHOLD = "g0_move_threshold"
CONF_G9_STILL_THRESHOLD = "g0_still_threshold"
CONF_G10_MOVE_THRESHOLD = "g1_move_threshold"
CONF_G10_STILL_THRESHOLD = "g1_still_threshold"
CONF_G11_MOVE_THRESHOLD = "g2_move_threshold"
CONF_G11_STILL_THRESHOLD = "g2_still_threshold"
CONF_G12_MOVE_THRESHOLD = "g3_move_threshold"
CONF_G12_STILL_THRESHOLD = "g3_still_threshold"
CONF_G13_MOVE_THRESHOLD = "g4_move_threshold"
CONF_G13_STILL_THRESHOLD = "g4_still_threshold"
CONF_G14_MOVE_THRESHOLD = "g5_move_threshold"
CONF_G14_STILL_THRESHOLD = "g5_still_threshold"
CONF_G15_MOVE_THRESHOLD = "g6_move_threshold"
CONF_G15_STILL_THRESHOLD = "g6_still_threshold"
CONF_TAKE_EFFECT_FRAMES = "take_effect_frames"
CONF_LOOSE_EFFECT_FRAMES = "loose_effect_frames"

DISTANCES = [0, 0.6, 1.2, 1.8, 2.4, 3, 3.6, 4.2, 4.8, 5.4, 6, 6.6, 7.2, 7.8, 8.4, 9]

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(LD2420Component),
            cv.Optional(CONF_DETECTION_GATE_MAX, default="6m"): cv.All(
                cv.distance, cv.one_of(*DISTANCES, float=True)
            ),
            cv.Optional(CONF_DETECTION_GATE_MIN, default="0.6m"): cv.All(
                cv.distance, cv.one_of(*DISTANCES, float=True)
            ),
            cv.Optional(CONF_TIMEOUT, default="5s"): cv.All(
                cv.positive_time_period_seconds,
                cv.Range(max=cv.TimePeriod(seconds=32767)),
            ),
            cv.Optional(CONF_PRESENCE_TIME_WINDOW, default="120s"): cv.All(
                cv.positive_time_period_seconds,
                cv.Range(max=cv.TimePeriod(seconds=32767)),
            ),
            cv.Optional(CONF_G0_MOVE_THRESHOLD, default=60000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G0_STILL_THRESHOLD, default=60000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G1_MOVE_THRESHOLD, default=50000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G1_STILL_THRESHOLD, default=50000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G2_MOVE_THRESHOLD, default=50000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G2_STILL_THRESHOLD, default=50000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G3_MOVE_THRESHOLD, default=40000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G3_STILL_THRESHOLD, default=40000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G4_MOVE_THRESHOLD, default=40000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G4_STILL_THRESHOLD, default=40000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G5_MOVE_THRESHOLD, default=40000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G5_STILL_THRESHOLD, default=40000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G6_MOVE_THRESHOLD, default=40000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G6_STILL_THRESHOLD, default=40000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G7_MOVE_THRESHOLD, default=40000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G7_STILL_THRESHOLD, default=40000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G8_MOVE_THRESHOLD, default=40000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G8_STILL_THRESHOLD, default=40000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G9_MOVE_THRESHOLD, default=40000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G9_STILL_THRESHOLD, default=30000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G10_MOVE_THRESHOLD, default=30000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G10_STILL_THRESHOLD, default=30000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G11_MOVE_THRESHOLD, default=20000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G11_STILL_THRESHOLD, default=20000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G12_MOVE_THRESHOLD, default=20000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G12_STILL_THRESHOLD, default=20000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G13_MOVE_THRESHOLD, default=20000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G13_STILL_THRESHOLD, default=20000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G14_MOVE_THRESHOLD, default=10000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G14_STILL_THRESHOLD, default=10000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G15_MOVE_THRESHOLD, default=10000): cv.int_range(
                min=0, max=65535
            ),
            cv.Optional(CONF_G15_STILL_THRESHOLD, default=10000): cv.int_range(
                min=0, max=65535
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ld2420",
    baud_rate=256000,
    require_tx=True,
    require_rx=True,
    parity="NONE",
    stop_bits=1,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_timeout(config[CONF_TIMEOUT]))
    cg.add(var.set_min_move_distance(int(config[CONF_DETECTION_GATE_MIN]/0.6)))
    cg.add(var.set_max_move_distance(int(config[CONF_DETECTION_GATE_MAX]/0.6)))
    cg.add(
        var.set_range_config(
            config[CONF_G0_MOVE_THRESHOLD],
            config[CONF_G0_STILL_THRESHOLD],
            config[CONF_G1_MOVE_THRESHOLD],
            config[CONF_G1_STILL_THRESHOLD],
            config[CONF_G2_MOVE_THRESHOLD],
            config[CONF_G2_STILL_THRESHOLD],
            config[CONF_G3_MOVE_THRESHOLD],
            config[CONF_G3_STILL_THRESHOLD],
            config[CONF_G4_MOVE_THRESHOLD],
            config[CONF_G4_STILL_THRESHOLD],
            config[CONF_G5_MOVE_THRESHOLD],
            config[CONF_G5_STILL_THRESHOLD],
            config[CONF_G6_MOVE_THRESHOLD],
            config[CONF_G6_STILL_THRESHOLD],
            config[CONF_G7_MOVE_THRESHOLD],
            config[CONF_G7_STILL_THRESHOLD],
            config[CONF_G8_MOVE_THRESHOLD],
            config[CONF_G8_STILL_THRESHOLD],
            config[CONF_G9_MOVE_THRESHOLD],
            config[CONF_G9_STILL_THRESHOLD],
            config[CONF_G10_MOVE_THRESHOLD],
            config[CONF_G10_STILL_THRESHOLD],
            config[CONF_G11_MOVE_THRESHOLD],
            config[CONF_G11_STILL_THRESHOLD],
            config[CONF_G12_MOVE_THRESHOLD],
            config[CONF_G12_STILL_THRESHOLD],
            config[CONF_G13_MOVE_THRESHOLD],
            config[CONF_G13_STILL_THRESHOLD],
            config[CONF_G14_MOVE_THRESHOLD],
            config[CONF_G14_STILL_THRESHOLD],
            config[CONF_G15_MOVE_THRESHOLD],
            config[CONF_G15_STILL_THRESHOLD],
        )
    )


CALIBRATION_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(LD2420Component),
    }
)
