

#ifdef __cplusplus
extern "C" {
#endif


__declspec(dllexport) void* createSnake(unsigned int* numLinksInput,double* linkLengthsInput, double* linkMassesInput);

__declspec(dllexport) void* createIIwa7kg();

__declspec(dllexport) void* createYouBot();

__declspec(dllexport) void* createIIwa14kg();

__declspec(dllexport) void* createLBR4();

__declspec(dllexport) void getMassMatrix(void* robotInstance, double* MOutput, double* QInput);

__declspec(dllexport) void getCoriolisAndGravityVector(void* robotInstance, double* c, double* g, double* QInput, double* QDotInput);

__declspec(dllexport) void getJacobian(void* robotInstance, double* JOutput, double* QInput, double* pointPositionInput, unsigned int* bodyIndexInput);

__declspec(dllexport) void getWorldCoordinates(void* robotInstance, double* xOutput, double* QInput,double* pointPositionInput, unsigned int* bodyIndexInput);

__declspec(dllexport) void getRotationMatrix(void* robotInstance, double* ROutput, double* QInput, unsigned int* bodyIndexInput, double* finalOrientation);

__declspec(dllexport) void getDOF(void* robotInstance, int* DOF);

__declspec(dllexport) void updateKinematics(void* robotInstance, double* Q, double* QDot, double* QDotDot);


#ifdef __cplusplus
}
#endif
