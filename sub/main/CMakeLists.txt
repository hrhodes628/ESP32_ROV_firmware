idf_component_register(SRCS "main.c"
                        INCLUDE_DIRS "."
                        REQUIRES board telemetry motor ibus
                        )

idf_build_set_property(COMPILE_OPTIONS "-Wno-address-of-packed-member" APPEND)
