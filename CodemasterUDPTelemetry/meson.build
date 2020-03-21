
telemetry_files = files(
  'CodemasterUDPTelemetry.cpp',
)

extra_deps = []

if host_machine.system() == 'windows'
  telemetry_files += files(
    'stdafx.cpp',
    'wgetopt.c',
    'Serial.cpp',
  )
  extra_deps += cc.find_library('ws2_32')
else
  telemetry_files += files(
    'LinuxSerial.cpp',
  )
endif

telemetry_files = executable(
  meson.project_name(),
  telemetry_files,
  c_args : [
    pre_args,
    c_vis_args, 
    no_override_init_args, 
  ],
  cpp_args : [
    pre_args,
    cpp_vis_args, 
  ],
  dependencies : [
    extra_deps,
    dep_pthread,
  ],
  include_directories : [inc_common],
  link_args : cc.get_supported_link_arguments(['-Wl,-Bsymbolic-functions', '-Wl,-z,relro']),
  install : true
)
