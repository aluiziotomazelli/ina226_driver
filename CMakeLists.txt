set(COMPONENT_NAME ina226_driver)

idf_component_register(
    SRCS 
        "src/ina226_driver.cpp"
    INCLUDE_DIRS 
        "include"
        "include/interfaces"
    REQUIRES 
        idf_hals
)

if(IDF_TARGET STREQUAL "linux")
    target_compile_options(${COMPONENT_LIB} PUBLIC --coverage)
    target_link_options(${COMPONENT_LIB} PUBLIC --coverage)
endif()
