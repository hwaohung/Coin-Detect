##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=hello
ConfigurationName      :=Debug
WorkspacePath          :=/home/johnny/Documents/Test
ProjectPath            :=/home/johnny/Documents/Test/hello
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=johnny
Date                   :=23/08/16
CodeLitePath           :=/home/johnny/.codelite
LinkerName             :=g++
SharedObjectLinkerName :=g++ -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=.o.d
PreprocessSuffix       :=.o.i
DebugSwitch            :=-gstab
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
OutputFile             :=$(IntermediateDirectory)/$(ProjectName)
Preprocessors          :=
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E 
ObjectsFileList        :="hello.txt"
PCHCompileFlags        :=
MakeDirCommand         :=mkdir -p
LinkOptions            :=  
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch)/usr/local/include/ $(IncludeSwitch)/usr/local/include/pcl-1.8/ $(IncludeSwitch)/usr/local/include/eigen3/ $(IncludeSwitch)/usr/local/include/vtk-7.0/ 
IncludePCH             := 
RcIncludePath          := 
Libs                   := $(LibrarySwitch)boost_system $(LibrarySwitch)boost_filesystem $(LibrarySwitch)boost_thread $(LibrarySwitch)boost_date_time $(LibrarySwitch)boost_iostreams $(LibrarySwitch)boost_serialization $(LibrarySwitch)boost_chrono $(LibrarySwitch)boost_atomic $(LibrarySwitch)boost_regex $(LibrarySwitch)pcl_io $(LibrarySwitch)pcl_common $(LibrarySwitch)pcl_features $(LibrarySwitch)pcl_search $(LibrarySwitch)pcl_keypoints $(LibrarySwitch)pcl_visualization $(LibrarySwitch)pcl_kdtree $(LibrarySwitch)pcl_filters $(LibrarySwitch)pcl_sample_consensus $(LibrarySwitch)mlpack $(LibrarySwitch)vtkRenderingContext2D-7.0 $(LibrarySwitch)vtkCommonMisc-7.0 $(LibrarySwitch)vtkRenderingAnnotation-7.0 $(LibrarySwitch)vtktiff-7.0 $(LibrarySwitch)vtkFiltersCore-7.0 $(LibrarySwitch)vtkImagingSources-7.0 $(LibrarySwitch)vtkIOCore-7.0 $(LibrarySwitch)vtkhdf5_hl-7.0 $(LibrarySwitch)vtkexpat-7.0 $(LibrarySwitch)vtkFiltersProgrammable-7.0 $(LibrarySwitch)vtkInfovisLayout-7.0 $(LibrarySwitch)vtkFiltersParallelImaging-7.0 $(LibrarySwitch)vtkFiltersStatistics-7.0 $(LibrarySwitch)vtkIOXML-7.0 $(LibrarySwitch)vtkIOExport-7.0 $(LibrarySwitch)vtkIOGeometry-7.0 $(LibrarySwitch)vtkFiltersParallel-7.0 $(LibrarySwitch)vtkpng-7.0 $(LibrarySwitch)vtkFiltersTexture-7.0 $(LibrarySwitch)vtkCommonMath-7.0 $(LibrarySwitch)vtkFiltersExtraction-7.0 $(LibrarySwitch)vtkDICOMParser-7.0 $(LibrarySwitch)vtkRenderingCore-7.0 $(LibrarySwitch)vtkImagingColor-7.0 $(LibrarySwitch)vtkFiltersGeneric-7.0 $(LibrarySwitch)vtkRenderingLOD-7.0 $(LibrarySwitch)vtkCommonColor-7.0 $(LibrarySwitch)vtkCommonTransforms-7.0 $(LibrarySwitch)vtkIOInfovis-7.0 $(LibrarySwitch)vtkalglib-7.0 $(LibrarySwitch)vtkglew-7.0 $(LibrarySwitch)vtkIOAMR-7.0 $(LibrarySwitch)vtkFiltersAMR-7.0 $(LibrarySwitch)vtkDomainsChemistryOpenGL2-7.0 $(LibrarySwitch)vtkImagingStatistics-7.0 $(LibrarySwitch)vtkRenderingVolume-7.0 $(LibrarySwitch)vtkIOEnSight-7.0 $(LibrarySwitch)vtkFiltersHyperTree-7.0 $(LibrarySwitch)vtkfreetype-7.0 $(LibrarySwitch)vtkIOParallel-7.0 $(LibrarySwitch)vtkFiltersImaging-7.0 $(LibrarySwitch)vtkRenderingVolumeOpenGL2-7.0 $(LibrarySwitch)vtkIOParallelXML-7.0 $(LibrarySwitch)vtkFiltersGeneral-7.0 $(LibrarySwitch)vtkImagingMath-7.0 $(LibrarySwitch)vtkRenderingOpenGL2-7.0 $(LibrarySwitch)vtkIOLSDyna-7.0 $(LibrarySwitch)vtkRenderingFreeType-7.0 $(LibrarySwitch)vtkexoIIc-7.0 $(LibrarySwitch)vtkIOMovie-7.0 $(LibrarySwitch)vtklibxml2-7.0 $(LibrarySwitch)vtkFiltersGeometry-7.0 $(LibrarySwitch)vtkjsoncpp-7.0 $(LibrarySwitch)vtkhdf5-7.0 $(LibrarySwitch)vtkInteractionStyle-7.0 $(LibrarySwitch)vtkFiltersHybrid-7.0 $(LibrarySwitch)vtkCommonExecutionModel-7.0 $(LibrarySwitch)vtkIOMINC-7.0 $(LibrarySwitch)vtksqlite-7.0 $(LibrarySwitch)vtkDomainsChemistry-7.0 $(LibrarySwitch)vtkmetaio-7.0 $(LibrarySwitch)vtkzlib-7.0 $(LibrarySwitch)vtkIOLegacy-7.0 $(LibrarySwitch)vtkCommonSystem-7.0 $(LibrarySwitch)vtkFiltersSMP-7.0 $(LibrarySwitch)vtkImagingMorphological-7.0 $(LibrarySwitch)vtkImagingHybrid-7.0 $(LibrarySwitch)vtkImagingCore-7.0 $(LibrarySwitch)vtkIOVideo-7.0 $(LibrarySwitch)vtkIOImage-7.0 $(LibrarySwitch)vtkViewsInfovis-7.0 $(LibrarySwitch)vtkNetCDF-7.0 $(LibrarySwitch)vtkCommonCore-7.0 $(LibrarySwitch)vtkViewsContext2D-7.0 $(LibrarySwitch)vtkverdict-7.0 $(LibrarySwitch)vtkFiltersModeling-7.0 $(LibrarySwitch)vtkViewsCore-7.0 $(LibrarySwitch)vtkIOSQL-7.0 $(LibrarySwitch)vtkNetCDF_cxx-7.0 $(LibrarySwitch)vtksys-7.0 $(LibrarySwitch)vtkjpeg-7.0 $(LibrarySwitch)vtkInteractionWidgets-7.0 $(LibrarySwitch)vtkChartsCore-7.0 $(LibrarySwitch)vtkoggtheora-7.0 $(LibrarySwitch)vtkRenderingImage-7.0 $(LibrarySwitch)vtkRenderingLabel-7.0 $(LibrarySwitch)vtkCommonComputationalGeometry-7.0 $(LibrarySwitch)vtkImagingFourier-7.0 $(LibrarySwitch)vtkFiltersVerdict-7.0 $(LibrarySwitch)vtkParallelCore-7.0 $(LibrarySwitch)vtkFiltersSelection-7.0 $(LibrarySwitch)vtkImagingStencil-7.0 $(LibrarySwitch)vtkGeovisCore-7.0 $(LibrarySwitch)vtkImagingGeneral-7.0 $(LibrarySwitch)vtkInteractionImage-7.0 $(LibrarySwitch)vtkRenderingContextOpenGL2-7.0 $(LibrarySwitch)vtkFiltersSources-7.0 $(LibrarySwitch)vtkIOPLY-7.0 $(LibrarySwitch)vtkIOImport-7.0 $(LibrarySwitch)vtkInfovisCore-7.0 $(LibrarySwitch)vtkIOExodus-7.0 $(LibrarySwitch)vtkproj4-7.0 $(LibrarySwitch)vtkFiltersFlowPaths-7.0 $(LibrarySwitch)vtkIONetCDF-7.0 $(LibrarySwitch)vtkIOXMLParser-7.0 $(LibrarySwitch)vtkCommonDataModel-7.0 
ArLibs                 :=  "libboost_system.so" "libboost_filesystem.so" "libboost_thread.so" "libboost_date_time.so" "libboost_iostreams.so" "libboost_serialization.so" "libboost_chrono.so" "libboost_atomic.so" "libboost_regex.so" "libpcl_io.so" "libpcl_common.so" "libpcl_features.so" "libpcl_search.so" "libpcl_keypoints.so" "libpcl_visualization.so" "libpcl_kdtree.so" "libpcl_filters.so" "libpcl_sample_consensus.so" "libmlpack.so" "libvtkRenderingContext2D-7.0.so" "libvtkCommonMisc-7.0.so" "libvtkRenderingAnnotation-7.0.so" "libvtktiff-7.0.so" "libvtkFiltersCore-7.0.so" "libvtkImagingSources-7.0.so" "libvtkIOCore-7.0.so" "libvtkhdf5_hl-7.0.so" "libvtkexpat-7.0.so" "libvtkFiltersProgrammable-7.0.so" "libvtkInfovisLayout-7.0.so" "libvtkFiltersParallelImaging-7.0.so" "libvtkFiltersStatistics-7.0.so" "libvtkIOXML-7.0.so" "libvtkIOExport-7.0.so" "libvtkIOGeometry-7.0.so" "libvtkFiltersParallel-7.0.so" "libvtkpng-7.0.so" "libvtkFiltersTexture-7.0.so" "libvtkCommonMath-7.0.so" "libvtkFiltersExtraction-7.0.so" "libvtkDICOMParser-7.0.so" "libvtkRenderingCore-7.0.so" "libvtkImagingColor-7.0.so" "libvtkFiltersGeneric-7.0.so" "libvtkRenderingLOD-7.0.so" "libvtkCommonColor-7.0.so" "libvtkCommonTransforms-7.0.so" "libvtkIOInfovis-7.0.so" "libvtkalglib-7.0.so" "libvtkglew-7.0.so" "libvtkIOAMR-7.0.so" "libvtkFiltersAMR-7.0.so" "libvtkDomainsChemistryOpenGL2-7.0.so" "libvtkImagingStatistics-7.0.so" "libvtkRenderingVolume-7.0.so" "libvtkIOEnSight-7.0.so" "libvtkFiltersHyperTree-7.0.so" "libvtkfreetype-7.0.so" "libvtkIOParallel-7.0.so" "libvtkFiltersImaging-7.0.so" "libvtkRenderingVolumeOpenGL2-7.0.so" "libvtkIOParallelXML-7.0.so" "libvtkFiltersGeneral-7.0.so" "libvtkImagingMath-7.0.so" "libvtkRenderingOpenGL2-7.0.so" "libvtkIOLSDyna-7.0.so" "libvtkRenderingFreeType-7.0.so" "libvtkexoIIc-7.0.so" "libvtkIOMovie-7.0.so" "libvtklibxml2-7.0.so" "libvtkFiltersGeometry-7.0.so" "libvtkjsoncpp-7.0.so" "libvtkhdf5-7.0.so" "libvtkInteractionStyle-7.0.so" "libvtkFiltersHybrid-7.0.so" "libvtkCommonExecutionModel-7.0.so" "libvtkIOMINC-7.0.so" "libvtksqlite-7.0.so" "libvtkDomainsChemistry-7.0.so" "libvtkmetaio-7.0.so" "libvtkzlib-7.0.so" "libvtkIOLegacy-7.0.so" "libvtkCommonSystem-7.0.so" "libvtkFiltersSMP-7.0.so" "libvtkImagingMorphological-7.0.so" "libvtkImagingHybrid-7.0.so" "libvtkImagingCore-7.0.so" "libvtkIOVideo-7.0.so" "libvtkIOImage-7.0.so" "libvtkViewsInfovis-7.0.so" "libvtkNetCDF-7.0.so" "libvtkCommonCore-7.0.so" "libvtkViewsContext2D-7.0.so" "libvtkverdict-7.0.so" "libvtkFiltersModeling-7.0.so" "libvtkViewsCore-7.0.so" "libvtkIOSQL-7.0.so" "libvtkNetCDF_cxx-7.0.so" "libvtksys-7.0.so" "libvtkjpeg-7.0.so" "libvtkInteractionWidgets-7.0.so" "libvtkChartsCore-7.0.so" "libvtkoggtheora-7.0.so" "libvtkRenderingImage-7.0.so" "libvtkRenderingLabel-7.0.so" "libvtkCommonComputationalGeometry-7.0.so" "libvtkImagingFourier-7.0.so" "libvtkFiltersVerdict-7.0.so" "libvtkParallelCore-7.0.so" "libvtkFiltersSelection-7.0.so" "libvtkImagingStencil-7.0.so" "libvtkGeovisCore-7.0.so" "libvtkImagingGeneral-7.0.so" "libvtkInteractionImage-7.0.so" "libvtkRenderingContextOpenGL2-7.0.so" "libvtkFiltersSources-7.0.so" "libvtkIOPLY-7.0.so" "libvtkIOImport-7.0.so" "libvtkInfovisCore-7.0.so" "libvtkIOExodus-7.0.so" "libvtkproj4-7.0.so" "libvtkFiltersFlowPaths-7.0.so" "libvtkIONetCDF-7.0.so" "libvtkIOXMLParser-7.0.so" "libvtkCommonDataModel-7.0.so" 
LibPath                := $(LibraryPathSwitch). $(LibraryPathSwitch)/usr/local/lib/ 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := ar rcus
CXX      := g++
CC       := gcc
CXXFLAGS :=  -g -O0 -Wall -std=gnu++11 $(Preprocessors)
CFLAGS   :=  -g -O0 -Wall  $(Preprocessors)
ASFLAGS  := 
AS       := as


##
## User defined environment variables
##
CodeLiteDir:=/usr/share/codelite
Objects0=$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IntermediateDirectory)/pcl_func.cpp$(ObjectSuffix) $(IntermediateDirectory)/mlpack_func.cpp$(ObjectSuffix) 



Objects=$(Objects0) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild MakeIntermediateDirs
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects0)  > $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

MakeIntermediateDirs:
	@test -d ./Debug || $(MakeDirCommand) ./Debug


$(IntermediateDirectory)/.d:
	@test -d ./Debug || $(MakeDirCommand) ./Debug

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/main.cpp$(ObjectSuffix): main.cpp $(IntermediateDirectory)/main.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/johnny/Documents/Test/hello/main.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/main.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/main.cpp$(DependSuffix): main.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/main.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/main.cpp$(DependSuffix) -MM main.cpp

$(IntermediateDirectory)/main.cpp$(PreprocessSuffix): main.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/main.cpp$(PreprocessSuffix)main.cpp

$(IntermediateDirectory)/pcl_func.cpp$(ObjectSuffix): pcl_func.cpp $(IntermediateDirectory)/pcl_func.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/johnny/Documents/Test/hello/pcl_func.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/pcl_func.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/pcl_func.cpp$(DependSuffix): pcl_func.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/pcl_func.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/pcl_func.cpp$(DependSuffix) -MM pcl_func.cpp

$(IntermediateDirectory)/pcl_func.cpp$(PreprocessSuffix): pcl_func.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/pcl_func.cpp$(PreprocessSuffix)pcl_func.cpp

$(IntermediateDirectory)/mlpack_func.cpp$(ObjectSuffix): mlpack_func.cpp $(IntermediateDirectory)/mlpack_func.cpp$(DependSuffix)
	$(CXX) $(IncludePCH) $(SourceSwitch) "/home/johnny/Documents/Test/hello/mlpack_func.cpp" $(CXXFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/mlpack_func.cpp$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/mlpack_func.cpp$(DependSuffix): mlpack_func.cpp
	@$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/mlpack_func.cpp$(ObjectSuffix) -MF$(IntermediateDirectory)/mlpack_func.cpp$(DependSuffix) -MM mlpack_func.cpp

$(IntermediateDirectory)/mlpack_func.cpp$(PreprocessSuffix): mlpack_func.cpp
	$(CXX) $(CXXFLAGS) $(IncludePCH) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/mlpack_func.cpp$(PreprocessSuffix)mlpack_func.cpp


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) -r ./Debug/


