#if defined CURAN_WINDOWS

#include <windows.h>
#include <stdio.h>
#include <tchar.h>
#include <iostream>

class ThreadPool{

};

class ProcHandler{
    ProcHandler(ThreadPool pool ,int argc, TCHAR *argv[] ){
        
        if(argc<2){
            std::cout << "this is a standalone process";
        }
        std::cout << "the process exists within a larger pool of processes";



    }



};

int foo(int argc, TCHAR *argv[] ){
    return 1;
}

int main(int argc, TCHAR *argv[] ){
    STARTUPINFO si;
    PROCESS_INFORMATION pi;

    ZeroMemory( &si, sizeof(si) );
    si.cb = sizeof(si);
    ZeroMemory( &pi, sizeof(pi) );
    std::cout << "Windows testing...\n";
    if( argc != 2 )
    {
        printf("Usage: %s [cmdline]\n", argv[0]);
        return 1;
    }

    // Start the child process. 
    if( !CreateProcess( NULL,   // No module name (use command line)
        argv[1],        // Command line
        NULL,           // Process handle not inheritable
        NULL,           // Thread handle not inheritable
        FALSE,          // Set handle inheritance to FALSE
        0,              // No creation flags
        NULL,           // Use parent's environment block
        NULL,           // Use parent's starting directory 
        &si,            // Pointer to STARTUPINFO structure
        &pi )           // Pointer to PROCESS_INFORMATION structure
    ) 
    {
        printf( "CreateProcess failed (%d).\n", GetLastError() );
        return 1;
    }
    printf("Starting to wait\n");
    // Wait until child process exits.
    WaitForSingleObject( pi.hProcess, 1000 );
    
    printf("Stopped waiting\n");
    TerminateProcess(pi.hProcess,3);
    // Close process and thread handles. 
    CloseHandle( pi.hProcess );
    CloseHandle( pi.hThread );
    printf("Terminating");
    return 0;
}

#elif defined CURAN_LINUX

#include <iostream>

int main(){
    std::cout << "Linux testing...\n";
    return 0;
}

#endif