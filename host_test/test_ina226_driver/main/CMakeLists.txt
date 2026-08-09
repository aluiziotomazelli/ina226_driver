idf_component_register(
    SRCS 
        "main.cpp"
        "test_ina226_driver.cpp"
    
    INCLUDE_DIRS
        "."
        "../../../external/idf_hals/mocks"
        
    REQUIRES 
        ina226_driver
        idf_hals
        gtest
        WHOLE_ARCHIVE
)

if(IDF_TARGET STREQUAL "linux")
    target_compile_options(${COMPONENT_LIB} PUBLIC --coverage)
    target_link_options(${COMPONENT_LIB} PUBLIC --coverage)
endif()
