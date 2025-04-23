import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate_ir
from esphome.const import CONF_ID, CONF_MODEL

CODEOWNERS = ["@mSapik"]
AUTO_LOAD = ["climate_ir"]

# Подключаем C++-класс и enum
mhi_ns = cg.esphome_ns.namespace("mhi_multi_ir")
MhiClimate = mhi_ns.class_("MhiClimate", climate_ir.ClimateIR)
Model = mhi_ns.enum("Model")

MODELS = {
    "ZJ":  Model.ZJ,
    "ZM":  Model.ZM,
    "ZMP": Model.ZMP,
    "ZEA": Model.ZEA,
}

CONFIG_SCHEMA = climate_ir.CLIMATE_IR_WITH_RECEIVER_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(MhiClimate),
    cv.Required(CONF_MODEL): cv.enum(MODELS),
})

async def to_code(config):
    # создаём переменную (ID уже знает C++-класс)
    var = cg.new_Pvariable(config[CONF_ID])
    # устанавливаем модель
    cg.add(var.set_model(config[CONF_MODEL]))
    # регистрируем климат-IR (он внутри зарегистрирует и компонент)
    await climate_ir.register_climate_ir(var, config)
