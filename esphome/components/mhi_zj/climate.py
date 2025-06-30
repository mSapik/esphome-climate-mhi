import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate_ir

CODEOWNERS = ["@mSapik"]
AUTO_LOAD = ['climate_ir']

mhi_ns = cg.esphome_ns.namespace('mhi_zj')
MhiClimate = mhi_ns.class_('MhiClimate', climate_ir.ClimateIR)

CONFIG_SCHEMA = climate_ir.climate_ir_with_receiver_schema(MhiClimate)

async def to_code(config):
    var = await climate_ir.new_climate_ir(config)
    await cg.register_component(var, config)