## -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

def build(bld):

    lte_module_dependencies = ['core', 'network', 'spectrum', 'stats', 'buildings', 'virtual-net-device','point-to-point','applications','internet','csma','lte']
    if (bld.env['ENABLE_EMU']):
        lte_module_dependencies.append('fd-net-device')
    module = bld.create_ns3_module('lte-rl', lte_module_dependencies)
    module.source = [
        'helper/lte-rl-helper.cc',
        # 'model/lte-rl-harq-phy.cc'
        ]

    module_test = bld.create_ns3_module_test_library('lte')
    module_test.source = [
        ]

    headers = bld(features='ns3header')
    headers.module = 'lte-rl'
    headers.source = [
        'helper/lte-rl-helper.h',
        # 'model/lte-rl-harq-phy.h'
        ]

    if (bld.env['ENABLE_EMU']):
        module.source.append ('helper/emu-epc-helper.cc')
        headers.source.append ('helper/emu-epc-helper.h')

    if (bld.env['ENABLE_EXAMPLES']):
      bld.recurse('examples')

    # bld.ns3_python_bindings()
