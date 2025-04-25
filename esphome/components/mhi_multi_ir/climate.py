import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate_ir
from esphome.const import CONF_ID

CODEOWNERS = ["@mSapik"]
AUTO_LOAD = ["climate_ir"]

mhi_ns = cg.esphome_ns.namespace("mhi_multi_ir")
MhiClimate = mhi_ns.class_("MhiClimate", climate_ir.ClimateIR)
Model = mhi_ns.enum("Model")
SetFanLevels = mhi_ns.enum("SetFanLevels")

MODELS = {"ZJ": Model.ZJ, "ZEA": Model.ZEA, "ZM": Model.ZM, "ZMP": Model.ZMP}
FAN_LEVELS = {"3": SetFanLevels.FAN_LEVELS_3, "4": SetFanLevels.FAN_LEVELS_4}

CONFIG_SCHEMA = climate_ir.CLIMATE_IR_WITH_RECEIVER_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(MhiClimate),
    cv.Required("model"): cv.enum(MODELS),
    cv.Optional("set_fan_levels", default="3"): cv.enum(FAN_LEVELS),
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    cg.add(var.set_model(MODELS[config["model"]]))
    cg.add(var.set_fan_levels(FAN_LEVELS[config["set_fan_levels"]]))
    await climate_ir.register_climate_ir(var, config)
