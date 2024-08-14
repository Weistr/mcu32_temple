#设置相关路径变量
SET(START_UP_DIR 
${PROJECT_SOURCE_DIR}/Lib/GD32_Lib/CMSIS/GD/GD32F3x0/Source/GCC)
SET(CORE_SUPPORT_DIR ${PROJECT_SOURCE_DIR}/Lib/GD32_Lib/CMSIS)
SET(PERIPHERALS_DIR ${PROJECT_SOURCE_DIR}/Lib/GD32_Lib/GD32F3x0_standard_peripheral)
#添加头文件搜索路径
include_directories(
    ${CORE_SUPPORT_DIR}
    ${CORE_SUPPORT_DIR}/GD/GD32F3x0/Include
    ${PERIPHERALS_DIR}/Include
    ${PROJECT_SOURCE_DIR}/App/Inc
    
    ${PROJECT_SOURCE_DIR}/drivers/Inc
)
#设置启动文件变量
SET(START_UP_ASM startup_gd32f3x0.S)
#设置启动文件 C 属性
set_property(SOURCE ${START_UP_DIR}/${START_UP_ASM} PROPERTY LANGUAGE C)
# GLOB 选项将会为所有匹配查询表达式的文件生成一个文件 list，并将该 list 存储进变量所定义的STD_LIB,SRC_CORE 里

file(GLOB STD_LIB ${PERIPHERALS_DIR}/Source/*.c)
file(GLOB SRC_CORE ${CORE_SUPPORT_DIR}/GD/GD32F3x0/Source/*.c)
#生成库目标 gd32_lib
add_library(gd32_lib 
${STD_LIB}
${SRC_CORE}
${START_UP_DIR}/${START_UP_ASM}
)
#设置库输出的名称
set_target_properties(gd32_lib PROPERTIES OUTPUT_NAME "gd32_lib")
#设置库文件的默认输出路径
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib)