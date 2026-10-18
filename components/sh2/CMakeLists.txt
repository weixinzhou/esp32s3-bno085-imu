# CEVA's sh2 driver, vendored unmodified from github.com/ceva-dsp/sh2.
# euler.c is not needed: sh2_SensorValue gives us the quaternion directly.
idf_component_register(
    SRCS "sh2.c" "shtp.c" "sh2_SensorValue.c" "sh2_util.c"
    INCLUDE_DIRS "."
)
