project('CodemasterUDPTelemetry',
  ['c', 'cpp'],
  version : '0.0.1',
  license : 'MIT',
  default_options : ['buildtype=release', 'c_std=c99', 'cpp_std=c++17']
)

cc = meson.get_compiler('c')
cpp = meson.get_compiler('cpp')
null_dep = dependency('', required : false)

pre_args = [
  '-D__STDC_CONSTANT_MACROS',
  '-D__STDC_FORMAT_MACROS',
  '-D__STDC_LIMIT_MACROS',
  '-DPACKAGE_VERSION="@0@"'.format(meson.project_version()),
]

# Define DEBUG for debug builds only (debugoptimized is not included on this one)
if get_option('buildtype') == 'debug'
  pre_args += '-DDEBUG'
else
  pre_args += '-DNDEBUG'
endif

# TODO: this is very incomplete
if ['linux', 'cygwin', 'gnu'].contains(host_machine.system())
  pre_args += '-D_GNU_SOURCE'
  pre_args += '-DHAVE_PTHREAD'
endif

if get_option('glibcxx_asserts')
  pre_args += '-D_GLIBCXX_ASSERTIONS'
endif

inc_common = [
    include_directories('CodemasterUDPTelemetry'),
]

# Check for generic C arguments
c_args = []
foreach a : ['-Werror=implicit-function-declaration',
             '-Werror=missing-prototypes', '-Werror=return-type',
             '-Werror=incompatible-pointer-types',
             '-fno-math-errno',
             '-fno-trapping-math', '-Qunused-arguments']
  if cc.has_argument(a)
    c_args += a
  endif
endforeach

foreach a : ['missing-field-initializers', 'format-truncation']
  if cc.has_argument('-W' + a)
    c_args += '-Wno-' + a
  endif
endforeach

c_vis_args = []
if cc.has_argument('-fvisibility=hidden')
  c_vis_args += '-fvisibility=hidden'
endif

# Check for generic C++ arguments
cpp_args = []
foreach a : ['-Werror=return-type',
             '-fno-math-errno', '-fno-trapping-math',
             '-Qunused-arguments']
  if cpp.has_argument(a)
    cpp_args += a
  endif
endforeach

# For some reason, the test for -Wno-foo always succeeds with gcc, even if the
# option is not supported. Hence, check for -Wfoo instead.

foreach a : ['non-virtual-dtor', 'missing-field-initializers', 'format-truncation']
  if cpp.has_argument('-W' + a)
    cpp_args += '-Wno-' + a
  endif
endforeach

no_override_init_args = []
foreach a : ['override-init', 'initializer-overrides']
  if cc.has_argument('-W' + a)
    no_override_init_args += '-Wno-' + a
  endif
endforeach

cpp_vis_args = []
if cpp.has_argument('-fvisibility=hidden')
  cpp_vis_args += '-fvisibility=hidden'
endif

foreach a : pre_args
  add_project_arguments(a, language : ['c', 'cpp'])
endforeach
foreach a : c_args
  add_project_arguments(a, language : ['c'])
endforeach
foreach a : cpp_args
  add_project_arguments(a, language : ['cpp'])
endforeach

dep_pthread = cc.find_library('pthread')

subdir('CodemasterUDPTelemetry')
