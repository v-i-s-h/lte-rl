## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):

    lte_module_dependencies = ['core', 'network', 'spectrum', 'stats', 'buildings', 'virtual-net-device','point-to-point','applications','internet','csma','lte']
    if (bld.env['ENABLE_EMU']):
        lte_module_dependencies.append('fd-net-device')
    module = bld.create_ns3_module('lte-rl', lte_module_dependencies)
    module.source = [
        'helper/lte-rl-helper.cc',
        'model/lte-rl-ue-phy.cc',
        'model/lte-rl-spectrum-phy.cc',
        'model/lte-rl-phy.cc',
        'model/lte-rl-ue-net-device.cc',
        'model/lte-rl-enb-net-device.cc',
        'model/lte-rl-enb-phy.cc'
        ]

    module_test = bld.create_ns3_module_test_library('lte')
    module_test.source = [
        ]

    headers = bld(features='ns3header')
    headers.module = 'lte-rl'
    headers.source = [
        'helper/lte-rl-helper.h',
        'model/lte-rl-ue-phy.h',
        'model/lte-rl-spectrum-phy.h',
        'model/lte-rl-phy.h',
        'model/lte-rl-ue-net-device.h',
        'model/lte-rl-enb-net-device.h',
        'model/lte-rl-enb-phy.h'
        ]

    # if (bld.env['ENABLE_EMU']):
    #     module.source.append ('helper/emu-epc-helper.cc')
    #     headers.source.append ('helper/emu-epc-helper.h')

    if (bld.env['ENABLE_EXAMPLES']):
      bld.recurse('examples')

    # bld.ns3_python_bindings()
