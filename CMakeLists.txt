cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(IMUble)

# 确保您的应用主文件 src/main.c 被明确包含
target_sources(app PRIVATE src/main.c)