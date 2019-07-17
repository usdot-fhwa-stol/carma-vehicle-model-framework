# test_libs Folder
This folder holds .so files for unit testing the ModelLoader class in the ModelLoaderTest.cpp and LibVehicleModelTest.cpp files. 

unittest_no_create_function_lib.so contains a library missing its "create" function entry point
unittest_no_destroy_function_lib.so contains a library missing its "destroy" function exit point
unittest_vehicle_model_shared_lib.so contains a valid vehicle model library which should load without exception

These files will need to be regenerated if the vehicle model interface changes
