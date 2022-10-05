clear mex % so that we can overwrite the mexw64/mexa64 files

disp('Building testceres')

% For dynamic linking，- and \ is the same
mex LINKFLAGS="$LINKFLAGS -LIBPATH:ceres-windows\x64\Release ceres.lib libglog_static.lib" ...
    COMPFLAGS="$COMPFLAGS -Wall...
    ... /D DEBUG ...
    -DGOOGLE_GLOG_DLL_DECL= -Iceres-windows\ceres-solver\include -Iceres-windows\win\include ...
    -Iceres-windows\glog\src -Iceres-windows\glog\src\windows -Iceres-windows\eigen" ...
    -Iceres-windows\Eigen\ ceres_optimization_allresiduals.cpp pose_local_parameterization.cpp utility.cpp

% % dynamic linking another mex method, but failed
% mex "$COMPFLAGS ...
%     ... /D DEBUG ...
%     /D GOOGLE_GLOG_DLL_DECL= ...
%     /I ceres-windows\ceres-solver\include /I ceres-windows\win\include ...
%     /I ceres-windows\glog\src /I ceres-windows\glog\src\windows /I ceres-windows\eigen" ...
%     -Lceres-windows\x64\Release -lceres -llibglog_static testceres.cpp 

% copyfile('ceres-windows\x64\Release\ceres.dll','.')
% https://ww2.mathworks.cn/matlabcentral/answers/99915-can-i-call-an-external-shared-library-dll-from-a-matlab-mex-file
% https://ww2.mathworks.cn/matlabcentral/answers/103012-how-can-i-call-a-dll-or-shared-library-function-using-mex-files-instead-of-the-generic-dll-shared
% /D is #define
