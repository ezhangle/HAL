# build the library
set( BUILD_Csv_Poses true CACHE BOOL force )

if( BUILD_Csv_Poses )
    add_to_hal_sources( CsvPosesDriver.h CsvPosesDriver.cpp CsvPosesFactory.cpp )
endif()
