set(HDRS
    PosesDevice.h
    PosesDriverInterface.h
)

add_to_hal_headers( ${HDRS} )

add_subdirectory( Drivers )
