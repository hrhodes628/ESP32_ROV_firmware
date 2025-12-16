idf_component_register(
                        INCLUDE_DIRS "include"
                        SRCS "control.c"
                        PRIV_REQUIRES motor ibus arming
)