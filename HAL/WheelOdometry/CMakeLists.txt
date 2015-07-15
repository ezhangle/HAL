set(HDRS
    WheelOdometryDevice.h
    WheelOdometryDriverInterface.h
)

add_to_hal_headers( ${HDRS} )

add_subdirectory( Drivers )
