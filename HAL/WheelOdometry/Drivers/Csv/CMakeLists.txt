# build the library
set( BUILD_Csv_WheelOdometry true CACHE BOOL force )

if( BUILD_Csv_WheelOdometry )
    add_to_hal_sources( CsvWheelOdometryDriver.h CsvWheelOdometryDriver.cpp CsvWheelOdometryFactory.cpp )
endif()
