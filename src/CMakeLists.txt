# --- Pipeline library ---
# All reusable code builds into a static lib, linked by the main executable and tests.

add_library(vision_lib STATIC
    pipeline/router.cpp
    pipeline/threadedStage.cpp
    pipeline/pipeline.cpp

    # Stages — add files here as you implement them
    stages/capture_stage.cpp
    # stages/OpticalFlowStage.cpp
    # stages/OrbStage.cpp
    # stages/MatchingStage.cpp
    # stages/RansacStage.cpp
    # stages/PoseStage.cpp
    # stages/OutputStage.cpp

    # DB
    # db/KeypointDB.cpp
    # db/DBSerializer.cpp

    # Utils
    # utils/Logger.cpp
    # utils/Config.cpp     <- header-only for now, add .cpp here if needed
)

target_link_libraries(vision_lib PUBLIC vision_common)

# --- Main executable ---
add_executable(vision main.cpp)
target_link_libraries(vision PRIVATE vision_lib)