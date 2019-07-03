if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
cet_add_compiler_flags(CXX -Wno-dynamic-exception-spec -Wno-return-type-c-linkage -Wno-undefined-var-template )
endif()

# canvas_utlities
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( $ENV{NUTOOLS_INC} )
include_directories( $ENV{IFDHC_FQ_DIR}/inc )
include_directories( $ENV{PANDORA_INC} )
include_directories( $ENV{LARPANDORACONTENT_INC} )

art_make( BASENAME_ONLY
    EXCLUDE 
    	CRTAnaFun.cc  
      BackTrackerTruthMatch.cxx
   	CRTXSEC_module.cc 
        CRTEventAnalyser_module.cc
        CRTtoTTree_module.cc
        CRTtoTTreeVector_module.cc
        EventAnalyser_module.cc
        CRTTPCAssoAnalyser_module.cc
        TPCTrackAnalyser_module.cc
        ##CRTLArPandoraExternalEventBuilding_module.cc
        ##CRTSimpleNeutrinoId_tool.cc
        ConsolidatedOutputAnalyser_module.cc
        ConsolidatedOutputAnalyserAllTracks_module.cc
        ConsolidatedOutputAnalyserSlices_module.cc
        NeutrinoCRTmatcher_module.cc
        NeutrinoCRTProducer_module.cc
        POTcount_module.cc
    LIB_LIBRARIES
        larcorealg_Geometry
        larcore_Geometry_Geometry_service
        ${ART_FRAMEWORK_CORE}
        ${ART_FRAMEWORK_IO_SOURCES}
        ${ART_FRAMEWORK_PRINCIPAL}
        art_Persistency_Common canvas
        art_Persistency_Provenance canvas
        art_Utilities canvas
	lardataobj_RecoBase
	lardataobj_AnalysisBase
  lardata_Utilities
  ##larpandora_LarPandoraEventBuilding
  larpandora_LarPandoraInterface
  ##larpandora_LArPandoraObjects
  ##larpandoraobj
  ##larpandora_LarPandoraEventBuilding
        ${ART_FRAMEWORK_SERVICES_REGISTRY}
        ${MF_MESSAGELOGGER}
        ${FHICLCPP}
        ${ROOT_BASIC_LIB_LIST}
        ${ROOT_GEOM}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}   
        cetlib cetlib_except
)

art_make_library( 
    LIBRARY_NAME 
        CRTAna
    SOURCE
        CRTAnaFun.cc      
    LIBRARIES 
        #larcorealg_Geometry
        #CRTData
        CRT
        bernfebdaq-core_Overlays
          art_Framework_Services_Optional
          art_Framework_Services_Registry
          art_Framework_Services_Optional_TFileService_service
          lardataobj_RecoBase
          ##lardataobj_AnalysisBase
          ${LARDATA_LIB}
          lardata_Utilities
        ${MF_MESSAGELOGGER}
        ${ART_FRAMEWORK_CORE}
		    ${ART_FRAMEWORK_PRINCIPAL}
        ${FHICLCPP}
        ${ROOT_BASIC_LIB_LIST}
        ${ROOT_GEOM}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}   
        cetlib cetlib_except
)

art_make_library( 
    LIBRARY_NAME 
        CRTBackTrack
    SOURCE
        BackTrackerTruthMatch.cxx      
    LIBRARIES 
          ##art_Persistency_Common canvas
        ##art_Persistency_Provenance canvas
        ##art_Utilities canvas     
        ##art_Persistency_Common canvas
        ##art_Persistency_Provenance canvas
        ##art_Utilities canvas
          ##larpandora_LArPandoraInterface
          ##lardataobj_RecoBase
          ##lardataobj_AnalysisBase
          lardataobj_RawData
          ##lardata_Utilities
          #larsim_Simulation lardataobj_Simulation
          #larsim_MCCheater_BackTrackerService_service
          #larsim_MCCheater_ParticleInventoryService_service
          ##nusimdata_SimulationBase
          #ubana_AnalysisTree_MCTruth_AssociationsTruth_tool
          #ubraw_RawData
          larcorealg_Geometry
        larcore_Geometry_Geometry_service
        ${ART_FRAMEWORK_CORE}
        ${ART_FRAMEWORK_IO_SOURCES}
        ${ART_FRAMEWORK_PRINCIPAL}
        art_Persistency_Common canvas
        art_Persistency_Provenance canvas
        art_Utilities canvas
	lardataobj_RecoBase
	lardataobj_AnalysisBase
  lardata_Utilities
  ##larpandora_LarPandoraEventBuilding
  ##larpandora_LarPandoraEventBuilding
  larpandora_LArPandoraInterface
  nusimdata_SimulationBase
  ##larpandora_LArPandoraObjects
  ##larpandoraobj
  ##larpandora_LarPandoraEventBuilding
        ${ART_FRAMEWORK_SERVICES_REGISTRY}
        ${MF_MESSAGELOGGER}
        ${FHICLCPP}
        ${ROOT_BASIC_LIB_LIST}
        ${ROOT_GEOM}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}   
        cetlib cetlib_except


)



simple_plugin(CRTXSEC "module"
            CRT
          bernfebdaq-core_Overlays
          art_Framework_Services_Optional
          art_Framework_Services_Registry
          art_Framework_Services_Optional_TFileService_service
          ${ROOT_BASIC_LIB_LIST} )
          
simple_plugin(POTcount "module"
          CRT
          bernfebdaq-core_Overlays
          art_Framework_Services_Optional
          art_Framework_Services_Registry
          art_Framework_Services_Optional_TFileService_service
          ${ROOT_BASIC_LIB_LIST} )          
          
simple_plugin(CRTEventAnalyser "module"
            CRT
            CRTAna
          bernfebdaq-core_Overlays
          art_Framework_Services_Optional
          art_Framework_Services_Registry
          art_Framework_Services_Optional_TFileService_service
          ${ROOT_BASIC_LIB_LIST} )
          
simple_plugin(CRTtoTTree "module"
            CRT
          bernfebdaq-core_Overlays
          art_Framework_Services_Optional
          art_Framework_Services_Registry
          art_Framework_Services_Optional_TFileService_service
          ${ROOT_BASIC_LIB_LIST} )
          
simple_plugin(CRTtoTTreeVector "module"
            CRT
            CRTAna
          bernfebdaq-core_Overlays
          art_Framework_Services_Optional
          art_Framework_Services_Registry
          art_Framework_Services_Optional_TFileService_service
          ${ROOT_BASIC_LIB_LIST} )
         
simple_plugin(EventAnalyser "module"
            CRT
            CRTAna
          bernfebdaq-core_Overlays
          #LArPandoraContent
          #larpandoraobj
          art_Framework_Services_Optional
          art_Framework_Services_Registry
          art_Framework_Services_Optional_TFileService_service
          ${ROOT_BASIC_LIB_LIST} )
        
simple_plugin(CRTTPCAssoAnalyser "module"
            CRT
            CRTAna
          bernfebdaq-core_Overlays
          art_Framework_Services_Optional
          art_Framework_Services_Registry
          art_Framework_Services_Optional_TFileService_service
          ${ROOT_BASIC_LIB_LIST} )
          
simple_plugin(TPCTrackAnalyser "module"
            CRT
            CRTAna
          bernfebdaq-core_Overlays
          art_Framework_Services_Optional
          art_Framework_Services_Registry
          art_Framework_Services_Optional_TFileService_service
          ${ROOT_BASIC_LIB_LIST} )
          
simple_plugin(ConsolidatedOutputAnalyserAllTracks "module"
            CRT
            CRTAna
          bernfebdaq-core_Overlays
          lardataobj_RecoBase
          lardataobj_AnalysisBase
          lardata_Utilities
          
          larcorealg_Geometry
                        larcore_Geometry_Geometry_service
                        #larsim_Simulation lardataobj_Simulation
                        lardataobj_RawData
                        lardataobj_RecoBase
                        lardataobj_AnalysisBase
                        lardata_ArtDataHelper
                        lardata_Utilities
                        ##larreco_RecoAlg_ClusterRecoUtil
                        ##larreco_RecoAlg
                        ##larreco_ClusterFinder
                        ##larreco_Calorimetry
                        ${PANDORASDK}
                        ${PANDORAMONITORING}
                        ##larpandora_LarPandoraEventBuilding
                        larpandora_LArPandoraInterface
                        nusimdata_SimulationBase
                        ${ART_FRAMEWORK_CORE}
                        ${ART_FRAMEWORK_PRINCIPAL}
                        ${ART_FRAMEWORK_SERVICES_REGISTRY}
                        ${ART_FRAMEWORK_SERVICES_BASIC}
                        ${ART_FRAMEWORK_SERVICES_OPTIONAL}
                        ${ART_FRAMEWORK_SERVICES_OPTIONAL_TFILESERVICE_SERVICE}
                        art_Persistency_Common
                        art_Persistency_Provenance
                        art_Utilities
                        canvas
                        ${MF_MESSAGELOGGER}
                        ##${FHICLCPP}##larreco_ClusterFinder
                        cetlib cetlib_except
                        ${Boost_SYSTEM_LIBRARY}
                        ${Boost_FILESYSTEM_LIBRARY}
                        ${ROOT_GEOM}
                        ${ROOT_BASIC_LIB_LIST}
                        ${ROOT_GENVECTOR}
                        ##MODULE_LIBRARIES larpandora_LArPandoraEventBuilding
          
         art_Framework_Services_Optional
          art_Framework_Services_Registry
          art_Framework_Services_Optional_TFileService_service
          ${ROOT_BASIC_LIB_LIST} 
          ##larpandora_LarPandoraEventBuilding 
          )
          
simple_plugin(NeutrinoCRTmatcher "module"
            CRT
            CRTAna
            CRTBackTrack
          bernfebdaq-core_Overlays
          lardataobj_RecoBase
          lardataobj_AnalysisBase
          lardata_Utilities
          
          larcorealg_Geometry
                        larcore_Geometry_Geometry_service
                        #larsim_Simulation lardataobj_Simulation
                        lardataobj_RawData
                        lardataobj_RecoBase
                        lardataobj_AnalysisBase
                        lardata_ArtDataHelper
                        lardata_Utilities
                        ##larreco_RecoAlg_ClusterRecoUtil
                        ##larreco_RecoAlg
                        ##larreco_ClusterFinder
                        ##larreco_Calorimetry
                        ${PANDORASDK}
                        ${PANDORAMONITORING}
                        ###larpandora_LArPandoraObjects
                        ##larpandora_LarPandoraEventBuilding
                        larpandora_LArPandoraInterface
                        nusimdata_SimulationBase
                        ${ART_FRAMEWORK_CORE}
                        ${ART_FRAMEWORK_PRINCIPAL}
                        ${ART_FRAMEWORK_SERVICES_REGISTRY}
                        ${ART_FRAMEWORK_SERVICES_BASIC}
                        ${ART_FRAMEWORK_SERVICES_OPTIONAL}
                        ${ART_FRAMEWORK_SERVICES_OPTIONAL_TFILESERVICE_SERVICE}
                        art_Persistency_Common
                        art_Persistency_Provenance
                        art_Utilities
                        canvas
                        ${MF_MESSAGELOGGER}
                        ${FHICLCPP}
                        cetlib cetlib_except
                        ${Boost_SYSTEM_LIBRARY}
                        ${Boost_FILESYSTEM_LIBRARY}
                        ${ROOT_GEOM}
                        ${ROOT_BASIC_LIB_LIST}
                        ${ROOT_GENVECTOR}
                        ##MODULE_LIBRARIES larpandora_LArPandoraEventBuilding
          
         art_Framework_Services_Optional
          art_Framework_Services_Registry
          art_Framework_Services_Optional_TFileService_service
          ${ROOT_BASIC_LIB_LIST} 
          ##larpandora_LarPandoraEventBuilding 
            )
            
simple_plugin(NeutrinoCRTProducer "module"
            CRT
            CRTAna
            CRTBackTrack
          bernfebdaq-core_Overlays
          lardataobj_RecoBase
          lardataobj_AnalysisBase
          lardata_Utilities
          
          larcorealg_Geometry
                        larcore_Geometry_Geometry_service
                        #larsim_Simulation lardataobj_Simulation
                        lardataobj_RawData
                        lardataobj_RecoBase
                        lardataobj_AnalysisBase
                        lardata_ArtDataHelper
                        lardata_Utilities
                        ##larreco_RecoAlg_ClusterRecoUtil
                        ##larreco_RecoAlg
                        ##larreco_ClusterFinder
                        ##larreco_Calorimetry
                        ${PANDORASDK}
                        ${PANDORAMONITORING}
                        ###larpandora_LArPandoraObjects
                        ##larpandora_LarPandoraEventBuilding
                        larpandora_LArPandoraInterface
                        nusimdata_SimulationBase
                        ${ART_FRAMEWORK_CORE}
                        ${ART_FRAMEWORK_PRINCIPAL}
                        ${ART_FRAMEWORK_SERVICES_REGISTRY}
                        ${ART_FRAMEWORK_SERVICES_BASIC}
                        ${ART_FRAMEWORK_SERVICES_OPTIONAL}
                        ${ART_FRAMEWORK_SERVICES_OPTIONAL_TFILESERVICE_SERVICE}
                        art_Persistency_Common
                        art_Persistency_Provenance
                        art_Utilities
                        canvas
                        ${MF_MESSAGELOGGER}
                        ${FHICLCPP}
                        cetlib cetlib_except
                        ${Boost_SYSTEM_LIBRARY}
                        ${Boost_FILESYSTEM_LIBRARY}
                        ${ROOT_GEOM}
                        ${ROOT_BASIC_LIB_LIST}
                        ${ROOT_GENVECTOR}
                        ##MODULE_LIBRARIES larpandora_LArPandoraEventBuilding
          
         art_Framework_Services_Optional
          art_Framework_Services_Registry
          art_Framework_Services_Optional_TFileService_service
          ${ROOT_BASIC_LIB_LIST} 
          ##larpandora_LarPandoraEventBuilding 
            )
            
simple_plugin(ConsolidatedOutputAnalyserSlices "module"
            CRT
            CRTAna
            CRTBackTrack
          bernfebdaq-core_Overlays
          lardataobj_RecoBase
          lardataobj_AnalysisBase
          lardata_Utilities
          
          larcorealg_Geometry
                        larcore_Geometry_Geometry_service
                        #larsim_Simulation lardataobj_Simulation
                        lardataobj_RawData
                        lardataobj_RecoBase
                        lardataobj_AnalysisBase
                        lardata_ArtDataHelper
                        lardata_Utilities
                        ##larreco_RecoAlg_ClusterRecoUtil
                        ##larreco_RecoAlg
                        ##larreco_ClusterFinder
                        ##larreco_Calorimetry
                        ${PANDORASDK}
                        ${PANDORAMONITORING}
                        ##larpandora_LarPandoraEventBuilding
                        larpandora_LArPandoraInterface
                        nusimdata_SimulationBase
                        ${ART_FRAMEWORK_CORE}
                        ${ART_FRAMEWORK_PRINCIPAL}
                        ${ART_FRAMEWORK_SERVICES_REGISTRY}
                        ${ART_FRAMEWORK_SERVICES_BASIC}
                        ${ART_FRAMEWORK_SERVICES_OPTIONAL}
                        ${ART_FRAMEWORK_SERVICES_OPTIONAL_TFILESERVICE_SERVICE}
                        art_Persistency_Common
                        art_Persistency_Provenance
                        art_Utilities
                        canvas
                        ${MF_MESSAGELOGGER}
                        ${FHICLCPP}
                        cetlib cetlib_except
                        ${Boost_SYSTEM_LIBRARY}
                        ${Boost_FILESYSTEM_LIBRARY}
                        ${ROOT_GEOM}
                        ${ROOT_BASIC_LIB_LIST}
                        ${ROOT_GENVECTOR}
                        ##MODULE_LIBRARIES larpandora_LArPandoraEventBuilding
          
         art_Framework_Services_Optional
          art_Framework_Services_Registry
          art_Framework_Services_Optional_TFileService_service
          ${ROOT_BASIC_LIB_LIST} 
          ##larpandora_LarPandoraEventBuilding 
            )
  
simple_plugin(ConsolidatedOutputAnalyser "module"
            CRT
            CRTAna
          bernfebdaq-core_Overlays
          lardataobj_RecoBase
          lardataobj_AnalysisBase
          lardata_Utilities
          
          larcorealg_Geometry
                        larcore_Geometry_Geometry_service
                        #larsim_Simulation lardataobj_Simulation
                        lardataobj_RawData
                        lardataobj_RecoBase
                        lardataobj_AnalysisBase
                        lardata_ArtDataHelper
                        lardata_Utilities
                        ##larreco_RecoAlg_ClusterRecoUtil
                        ##larreco_RecoAlg
                        ##larreco_ClusterFinder
                        ##larreco_Calorimetry
                        ${PANDORASDK}
                        ${PANDORAMONITORING}
                        ##larpandora_LarPandoraEventBuilding
                        larpandora_LArPandoraInterface
                        nusimdata_SimulationBase
                        ${ART_FRAMEWORK_CORE}
                        ${ART_FRAMEWORK_PRINCIPAL}
                        ${ART_FRAMEWORK_SERVICES_REGISTRY}
                        ${ART_FRAMEWORK_SERVICES_BASIC}
                        ${ART_FRAMEWORK_SERVICES_OPTIONAL}
                        ${ART_FRAMEWORK_SERVICES_OPTIONAL_TFILESERVICE_SERVICE}
                        art_Persistency_Common
                        art_Persistency_Provenance
                        art_Utilities
                        canvas
                        ${MF_MESSAGELOGGER}
                        ${FHICLCPP}
                        cetlib cetlib_except
                        ${Boost_SYSTEM_LIBRARY}
                        ${Boost_FILESYSTEM_LIBRARY}
                        ${ROOT_GEOM}
                        ${ROOT_BASIC_LIB_LIST}
                        ${ROOT_GENVECTOR}
                        ##MODULE_LIBRARIES larpandora_LArPandoraEventBuilding
          
         art_Framework_Services_Optional
          art_Framework_Services_Registry
          art_Framework_Services_Optional_TFileService_service
          ${ROOT_BASIC_LIB_LIST} 
          ##larpandora_LarPandoraEventBuilding 
          )
         

          
install_headers()
install_fhicl()
install_source()


