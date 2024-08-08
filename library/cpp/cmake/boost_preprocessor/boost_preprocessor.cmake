include(FetchContent)
FetchContent_Declare(BoostPreprocessor
    GIT_REPOSITORY https://github.com/boostorg/preprocessor.git
    GIT_TAG boost-1.71.0
    GIT_SHALLOW ON)
FetchContent_MakeAvailable(BoostPreprocessor)
