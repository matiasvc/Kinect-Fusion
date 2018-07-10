files = ["cmake-build-debug/TSDF0.csv", "cmake-build-debug/TSDF1.csv"];
camFile = "cmake-build-debug/cams.csv"
%surfaceFiles(files)
%  surfaceFiles(files(1))
%  surfaceFiles(files(2))
% 
%surfaceFiles("cmake-build-debug/TSDF.csv")

scatterFiles("cmake-build-debug/TSDF.csv", camFile)
% scatterFiles(files, camFile)
% scatterFiles(files(1), camFile)
% scatterFiles(files(2), camFile)