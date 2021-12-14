from setuptools import setup, Extension

extension = Extension(
    'vl53l0jp_python',
    define_macros=[],
    include_dirs=['.', 'Api/core/inc', 'platform/inc'],
    libraries=[],
    library_dirs=[],
    sources=['Api/core/src/vl53lx_api.c',
             'Api/core/src/vl53lx_api_calibration.c',
             'Api/core/src/vl53lx_api_core.c',
             'Api/core/src/vl53lx_api_debug.c',
             'Api/core/src/vl53lx_api_preset_modes.c',
             'Api/core/src/vl53lx_core.c',
             'Api/core/src/vl53lx_core_support.c',
             'Api/core/src/vl53lx_dmax.c',
             'Api/core/src/vl53lx_hist_algos_gen3.c',
             'Api/core/src/vl53lx_hist_algos_gen4.c',
             'Api/core/src/vl53lx_hist_char.c',
             'Api/core/src/vl53lx_hist_core.c',
             'Api/core/src/vl53lx_hist_funcs.c',
             'Api/core/src/vl53lx_nvm.c',
             'Api/core/src/vl53lx_nvm_debug.c',
             'Api/core/src/vl53lx_register_funcs.c',
             'Api/core/src/vl53lx_sigma_estimate.c',
             'Api/core/src/vl53lx_silicon_core.c',
             'Api/core/src/vl53lx_wait.c',
             'Api/core/src/vl53lx_xtalk.c',
             'platform/src/vl53lx_platform.c',
             'python_lib/vl53l0x_python.c'])

setup(name='VL53LX',
      version='2.0.4',
      description='VL53LX sensor for raspberry PI/JetsonTX2',
      # author='JPT Embedded Solutions',
      # author_email='?',
      #url='https://github.com/pimoroni/VL53L0X-python',
      long_description='''
VL53LX sensor for raspberry PI.
''',
      ext_modules=[extension],
      package_dir={'': 'python'},
      py_modules=['VL53LX'],
      requires=['smbus' or 'smbus2'])
