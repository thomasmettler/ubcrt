/**
 *  @file   larpandora/LArPandoraEventBuilding/ConsolidatedOutputAnalyserAllTracks.cc
 *
 *  @brief  module for lar pandora external event building
 */

//#include "art/Framework/Core/EDProducer.h"
#include "art/Framework/Core/EDAnalyzer.h"
#include "art/Framework/Core/ModuleMacros.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "art/Framework/Principal/Run.h"
#include "art/Framework/Principal/SubRun.h"
#include "art/Utilities/make_tool.h"

#include "canvas/Utilities/InputTag.h"
#include "fhiclcpp/ParameterSet.h"
#include "lardata/Utilities/AssociationUtil.h"
#include "ubcrt/CRTXSEC/CRTAnaFun.hh"

#include "messagefacility/MessageLogger/MessageLogger.h"
#include "art/Framework/Services/Optional/TFileService.h"

#include "larpandora/LArPandoraInterface/LArPandoraHelper.h"
#include "larpandora/LArPandoraEventBuilding/Slice.h"
//#include "larpandora/LArPandoraEventBuilding/NeutrinoIdBaseTool.h"
//#include "ubcrt/CRTXSEC/CRTNeutrinoIdBaseTool.h"
#include "larpandora/LArPandoraEventBuilding/LArPandoraEvent.h"

#include "lardataobj/RecoBase/PFParticle.h"
//#include "larpandora/LArPandoraObjects/PFParticleMetadata.h"
#include "lardataobj/RecoBase/PFParticleMetadata.h"

#include "ubcrt/CRTXSEC/CRTAnaFun.hh"
#include "ubobj/CRT/CRTHit.hh"
#include "ubobj/CRT/CRTTrack.hh"
#include "ubobj/CRT/CRTTzero.hh"
#include "ubcrt/CRT/CRTAuxFunctions.hh"
#include "ubobj/RawData/DAQHeaderTimeUBooNE.h"

#include "lardataobj/RecoBase/Track.h"                                                                
#include "lardataobj/RecoBase/Hit.h"     
#include "lardataobj/RecoBase/MCSFitResult.h"
#include "lardataobj/AnalysisBase/T0.h" 
#include "lardataobj/AnalysisBase/ParticleID.h" 
#include "lardataobj/AnalysisBase/CosmicTag.h"                                                        
#include "lardataobj/AnalysisBase/Calorimetry.h"                                                      
#include "lardataobj/MCBase/MCTrack.h"                                                                
#include "lardataobj/RecoBase/OpFlash.h"                                                              
#include "lardata/Utilities/AssociationUtil.h"
#include "lardataobj/RecoBase/PFParticle.h"

#include "TTree.h"
#include "TBenchmark.h"
#include "TRandom.h"
#include "TSystem.h"
#include "TClonesArray.h"
#include "TObject.h"


namespace lar_pandora
{

class ConsolidatedOutputAnalyserAllTracks : public art::EDAnalyzer
{
public:
    explicit ConsolidatedOutputAnalyserAllTracks(fhicl::ParameterSet const & pset);
    
    ConsolidatedOutputAnalyserAllTracks(ConsolidatedOutputAnalyserAllTracks const &) = delete;
    ConsolidatedOutputAnalyserAllTracks(ConsolidatedOutputAnalyserAllTracks &&) = delete;
    ConsolidatedOutputAnalyserAllTracks & operator = (ConsolidatedOutputAnalyserAllTracks const &) = delete;
    ConsolidatedOutputAnalyserAllTracks & operator = (ConsolidatedOutputAnalyserAllTracks &&) = delete;

    void analyze(art::Event const &evt) override;
    
    // Selected optional functions.
    void beginJob() override;
    void endJob() override;

private:
    typedef std::map<art::Ptr<recob::PFParticle>, art::Ptr<larpandoraobj::PFParticleMetadata> > PFParticleToMetadata;

    art::ServiceHandle<art::TFileService> tfs;
    TTree * my_event_;
    std::vector<crt::CRTHit> crthit_vec;
    std::vector<crt::CRTTrack> crttrack_vec;
    std::vector<crt::CRTTzero> crtt0_vec;
    std::vector<recob::OpFlash> flash_vec;
    std::vector<anab::Calorimetry> calo_vec;
    std::vector<recob::MCSFitResult> MCSfit_vec;
    
    int crthit_counter; // # crt hits assigned to a tpc track
    int crttrack_counter; // # crt tracks assigned to a tpc track
    int crtt0_counter;  // # crt t0 assignet to a tpc track
    int track_counter = 0; // TPC track counter
    int track_nopandora_counter = 0;
    int crthit_moreflash_counter = 0;
    int crthit_noflash_counter = 0;
    int crthit_tot_counter = 0;
    
    int event_counter = 0;  //event counter
    int calo_counter = 0;
    int mcs_counter = 0;
    
    int slice_counter = 0; // slice id (#) in the event
    int max_track_nr = 0; // track multiplicity
    int max_shower_nr = 0; //shower multiplicity
    int max_slice = 0; // number of slices in the event
    
    double nuScore_ = 0;  // the neutrino score of the slice
    double nuScore_max =0; // the maximum neutrino score of any slice in the event
    
    recob::Track tpctrack_match;
    //double tpc_crthit_dist;
    double track_length;
    uint32_t fEvtNum; //Number of current event                       
    uint32_t frunNum;                //Run Number taken from event  
    uint32_t fsubRunNum;             //Subrun Number taken from event 

    double fTriTim_sec = 0;
    double fTriTim_nsec = 0;
    double fAbsTimFla = 0;
    double flash_PE = 0;
    double flash_y = 0;
    double flash_z = 0;

    double crthit_ts0 = 0;
    double crttrack_ts0 = 0;
    double crthit_ts1 = 0;
    double crttrack_ts1 = 0;
    
    double crthit_x;
    double crthit_y;
    double crthit_z;
    
    double crttrack_start_x;
    double crttrack_start_y;
    double crttrack_start_z;
    double crttrack_end_x;
    double crttrack_end_y;
    double crttrack_end_z;
    
    int fHardDelay_;
    int fCRTT0off_;
    
    double track_start_x;
    double track_start_y;
    double track_start_z;

    double track_end_x;
    double track_end_y;
    double track_end_z;

    double start_mom_x;
    double start_mom_y;
    double start_mom_z;
    double start_mom_tot;
    
    double track_mcs_mom;
    double end_mom_x;
    double end_mom_y;
    double end_mom_z;
    double theta_track;
    double phi_track;
    double zenith_angle;
    double azimuth_angle;

    /**
     *  @brief  Collect PFParticles from the ART event and their mapping to metadata objects
     *  @param  evt the ART event
     *  @param  particlesToMetadata the output mapping from PFParticles to their metadata
     *  @param  particles the output vector of particles
     */
    void CollectPFParticles(const art::Event &evt, PFParticleToMetadata &particlesToMetadata, PFParticleVector &particles) const;

    /**
     *  @brief  Build mapping from ID to PFParticle for fast navigation through the hierarchy
     *  @param  particlesToMetadata the input mapping from PFParticles to their metadata
     *  @param  particleMap the output mapping from ID to PFParticle
     */
    void BuildPFParticleMap(const PFParticleToMetadata &particlesToMetadata, PFParticleMap &particleMap) const;

    /**
     *  @brief  Collect PFParticles that have been identified as clear cosmic ray muons by pandora
     *  @param  allParticles input vector of all particles
     *  @param  particlesToMetadata the input mapping from PFParticles to their metadata
     *  @param  particleMap the input mapping from ID to PFParticle
     *  @param  clearCosmics the output vector of clear cosmic rays
     */
    void CollectClearCosmicRays(const PFParticleVector &allParticles, const PFParticleToMetadata &particlesToMetadata, const PFParticleMap &particleMap, PFParticleVector &clearCosmics) const;

    /**
     *  @brief  Collect slices 
     *  @param  allParticles input vector of all particles
     *  @param  particlesToMetadata the input mapping from PFParticles to their metadata
     *  @param  particleMap the input mapping from ID to PFParticle
     *  @param  slices the output vector of slices
     */
    void CollectSlices(const PFParticleVector &allParticles, const PFParticleToMetadata &particlesToMetadata, const PFParticleMap &particleMap, SliceVector &slices) const;

    /**
     *  @brief  Get the consolidated collection of particles based on the slice ids
     *  @param  allParticles input vector of all particles
     *  @param  clearCosmics the input vector of clear cosmic ray muons
     *  @param  slices the input vector of slices
     *  @param  consolidatedParticles the output vector of particles to include in the consolidated output 
     */
    void CollectConsolidatedParticles(const PFParticleVector &allParticles, const PFParticleVector &clearCosmics, const SliceVector &slices, PFParticleVector &consolidatedParticles) const;

    /**
     *  @brief  Query a metadata object for a given key and return the corresponding value
     *  @param  metadata the metadata object to query
     *  @param  key the key to search for
     *  @return the value in the metadata corresponding to the input key
     */
    float GetMetadataValue(const art::Ptr<larpandoraobj::PFParticleMetadata> &metadata, const std::string &key) const;

    void initialize_tmyevent();
    
    int verbose_;
    int saveTTree_ = 0;
    
    int track_selection = -1;

    std::string                         m_inputProducerLabel;  ///< Label for the Pandora instance that produced the collections we want to consolidated
    std::string                         m_crthitLabel;
    std::string                         data_label_assohits_;
    std::string                         data_label_assotracks_; 
    std::string                         data_label_assoCRTT0_;
    std::string                         data_label_CRTtzero_;
    std::string                         data_label_DAQHeader_;
    std::string                         data_label_flash_;
    std::string                         data_label_trackForMum_;
    std::string                         data_label_assotrackForMum_;
    std::string                         data_label_MCSfit_;
    std::string                         m_trackProducerLabel;  ///< Label for the track producer using the Pandora instance that produced the collections we want to consolidate
    std::string                         m_showerProducerLabel; ///< Label for the shower producer using the Pandora instance that produced the collections we want to consolidate
    std::string                         m_hitProducerLabel;    ///< Label for the hit producer that was used as input to the Pandora instance specified
    bool                                m_shouldProduceT0s;    ///< If we should produce T0s (relevant when stitching over multiple drift volumes)
    art::InputTag                       m_pandoraTag;          ///< The input tag for the pandora producer
    //std::unique_ptr<CRTNeutrinoIdBaseTool> m_neutrinoIdTool;      ///< The neutrino id tool
};

DEFINE_ART_MODULE(ConsolidatedOutputAnalyserAllTracks)

} // namespace lar_pandora

//------------------------------------------------------------------------------------------------------------------------------------------
// implementation follows

#include "Pandora/PdgTable.h"

namespace lar_pandora
{

ConsolidatedOutputAnalyserAllTracks::ConsolidatedOutputAnalyserAllTracks(fhicl::ParameterSet const &pset) :
  EDAnalyzer(pset),
    m_inputProducerLabel(pset.get<std::string>("InputProducerLabel")),
    m_crthitLabel(pset.get<std::string>("CRTHitLabel")),
    m_trackProducerLabel(pset.get<std::string>("TrackProducerLabel")),
    m_showerProducerLabel(pset.get<std::string>("ShowerProducerLabel")),
    m_hitProducerLabel(pset.get<std::string>("HitProducerLabel")),
    m_shouldProduceT0s(pset.get<bool>("ShouldProduceT0s")),
    m_pandoraTag(art::InputTag(m_inputProducerLabel))
    //m_neutrinoIdTool(art::make_tool<CRTNeutrinoIdBaseTool>(pset.get<fhicl::ParameterSet>("CRTNeutrinoIdTool")))
{
    //control variables
    verbose_ = pset.get<int>("verbose");
    saveTTree_ = pset.get<int>("saveTTree");
    track_selection = pset.get<int>("track_selection");
    //datalabels
    data_label_CRTtzero_ = pset.get<std::string>("data_label_CRTtzero");
    data_label_assohits_ = pset.get<std::string>("data_label_assohits");
    data_label_assoCRTT0_ = pset.get<std::string>("data_label_assoCRTT0");
    data_label_assotracks_ = pset.get<std::string>("data_label_assotracks"); 
    data_label_DAQHeader_ = pset.get<std::string>("data_label_DAQHeader");
    data_label_flash_ = pset.get<std::string>("data_label_flash");
    data_label_trackForMum_ =  pset.get<std::string>("data_label_trackForMum");
    data_label_MCSfit_ = pset.get<std::string>("data_label_MCSfit");
    data_label_assotrackForMum_ =  pset.get<std::string>("data_label_assotrackForMum");
    // crt variables
    fHardDelay_ = pset.get<int>("fHardDelay",40000);
    fCRTT0off_ = pset.get<int>("fCRTT0off",69000);
    
    
}

//------------------------------------------------------------------------------------------------------------------------------------------

void ConsolidatedOutputAnalyserAllTracks::analyze(art::Event const &evt)
{
  
  /*
  anab::ParticleID my_particleid;
  std::cout << "Size of my_particleid: " << sizeof(my_particleid) << std::endl;*/
  
  //anab::Calorimetry my_Calo;
  //std::cout << "Size of my_Calo: " << sizeof(my_Calo) << std::endl;
  //recob::MCSFitResult my_MCS;
  //std::cout << "Size of my_MCS: " << sizeof(my_MCS) << std::endl;
  
  
  
    std::cout << "Prozessing event nr: " << event_counter << std::endl;
    if(verbose_!=0) std::cout << "Run " << evt.run() << ", subrun " << evt.subRun() << std::endl;
    frunNum    = evt.run();
    fsubRunNum = evt.subRun();
    fEvtNum = evt.event();
    event_counter++;

    art::Handle< raw::DAQHeaderTimeUBooNE > rawHandle_DAQHeader;
    evt.getByLabel(data_label_DAQHeader_, rawHandle_DAQHeader);

    //check to make sure the data we asked for is valid                                                                                          
    if(!rawHandle_DAQHeader.isValid()){
      std::cout << "Run " << evt.run() << ", subrun " << evt.subRun()
                << ", event " << evt.event() << " has zero"
                << " DAQHeaderTimeUBooNE  " << " in with label " << data_label_DAQHeader_ << std::endl;
      return;
    }  

    raw::DAQHeaderTimeUBooNE const& my_DAQHeader(*rawHandle_DAQHeader);

    art::Timestamp evtTimeGPS = my_DAQHeader.gps_time();  
    //double evt_timeGPS_sec = evtTimeGPS.timeHigh();
    //double evt_timeGPS_nsec = evtTimeGPS.timeLow();
    fTriTim_sec = evtTimeGPS.timeHigh();
    fTriTim_nsec = evtTimeGPS.timeLow();
  
    art::Handle< std::vector<recob::OpFlash> > rawHandle_OpFlash;
    evt.getByLabel(data_label_flash_, rawHandle_OpFlash);
    std::vector<recob::OpFlash> const& OpFlashCollection(*rawHandle_OpFlash);   
    //get Optical Flash
    //NrFlash = OpFlashCollection.size();
    if(verbose_!=0) {
      std::cout << "Run " << evt.run() << ", subrun " << evt.subRun()
              << ", event " << evt.event() << " has " << "\033[32m" << OpFlashCollection.size() << "\033[0m"
              << " TPCFlashes " << " in module " << data_label_flash_ << std::endl;
    }
    
    PFParticleVector particles;
    PFParticleToMetadata particlesToMetadata;
    this->CollectPFParticles(evt, particlesToMetadata, particles);

    PFParticleMap particleMap;
    this->BuildPFParticleMap(particlesToMetadata, particleMap);

    PFParticleVector clearCosmics;
    this->CollectClearCosmicRays(particles, particlesToMetadata, particleMap, clearCosmics);
    std::vector< art::Ptr<recob::Track> > tracks_clearcosmic;
    std::vector< art::Ptr<recob::Shower> > showers_clearcosmic;
    art::Handle<std::vector<recob::PFParticle> > pfParticleHandle;
    evt.getByLabel(m_inputProducerLabel, pfParticleHandle);
    crtana::auxfunc::CollectTracksAndShowers(clearCosmics, pfParticleHandle, evt, tracks_clearcosmic, showers_clearcosmic, m_trackProducerLabel, m_showerProducerLabel);


    SliceVector slices;
    this->CollectSlices(particles, particlesToMetadata, particleMap, slices);
    //std::cout << "################Her I am...#################################" << std::endl;
    //m_neutrinoIdTool->ClassifySlicesCRTHit(slices, evt, m_crthitLabel);
    // tag neutrino candidates ///////////////////////////////////////////
    max_slice = slices.size();
    if(verbose_!=0) std::cout << "Number of Slices: " << max_slice << std::endl;
    //if (slices.empty()) return;
    /*
    // Find the most probable slice
    float highestNuScore(-std::numeric_limits<float>::max());
    unsigned int mostProbableSliceIndex(std::numeric_limits<unsigned int>::max());
    */
    // prepare all objects you want to use (track, CRT hits, crt tracks,crt t0..
    //art::Handle<std::vector<recob::PFParticle> > pfParticleHandle;
    //evt.getByLabel(m_inputProducerLabel, pfParticleHandle);
    art::Handle<std::vector<recob::Track> > rawHandle_TPCtrack;
    evt.getByLabel(m_trackProducerLabel, rawHandle_TPCtrack);
  
    art::Handle<std::vector<recob::Track> > rawHandle_TPCtrackMum;
    evt.getByLabel(data_label_trackForMum_, rawHandle_TPCtrackMum);
  
    art::FindMany<crt::CRTTrack> trk_crttrack_assn_v(rawHandle_TPCtrack, evt, data_label_assotracks_);
    // to grab the crt hits associated to a tpc track we need to go over crt t0 objects
    // since the association is only between tpc tracks - crt t0 and crt t0 - crt hits
    art::FindMany<crt::CRTTzero> trk_crtT0_assn_v(rawHandle_TPCtrack, evt, data_label_assoCRTT0_);

    art::Handle< std::vector<crt::CRTTzero> > rawHandle_CRTtzero;
    evt.getByLabel(data_label_CRTtzero_, rawHandle_CRTtzero);
    std::vector<crt::CRTTzero> const& CRTT0Collection(*rawHandle_CRTtzero);
    art::FindMany<crt::CRTHit> trk_crthit_assn_v(rawHandle_CRTtzero, evt, data_label_assohits_);
  
    slice_counter = 0;
    nuScore_ = 0;
    nuScore_max =0;
    max_track_nr = 0;
    max_shower_nr = 0;
    //art::Handle< std::vector<recob::Track> > rawHandle_Track;
    //evt.getByLabel(data_label_track_, rawHandle_Track);
    
  //std::vector<recob::Track> const& TrackCollection(*rawHandle_TPCtrack);   
  std::vector< art::Ptr<recob::Track> > TrackCollection;   
  
  //std::vector<recob::Track> TrackCollection; 
  if(track_selection == 0 || track_selection ==3){
    TrackCollection.insert(TrackCollection.end(), tracks_clearcosmic.begin(), tracks_clearcosmic.end());
  }
  
  if(track_selection !=0 ){
    for(unsigned int sliceIndex = 0; sliceIndex < slices.size(); sliceIndex++){
      PFParticleVector nu_pfparticle;
      slice_counter = sliceIndex+1;
      if(track_selection == 1) nu_pfparticle = slices.at(sliceIndex).GetTargetHypothesis();
      if(track_selection == 2 || track_selection ==3) nu_pfparticle = slices.at(sliceIndex).GetCosmicRayHypothesis();
      std::vector< art::Ptr<recob::Track> > tracks_slice;
      std::vector< art::Ptr<recob::Shower> > showers_slice;
      crtana::auxfunc::CollectTracksAndShowers(nu_pfparticle, pfParticleHandle, evt, tracks_slice, showers_slice, m_trackProducerLabel, m_showerProducerLabel);
      max_track_nr = tracks_slice.size();
      max_shower_nr = showers_slice.size();
      if(verbose_ !=0 )std::cout << "Track and Shower multiplicity: " << max_track_nr  << " - " << max_shower_nr << std::endl;
      
      TrackCollection.insert(TrackCollection.end(), tracks_slice.begin(), tracks_slice.end());

    }
  }
  
  std::cout << "Size of track_vector: " << TrackCollection.size() << std::endl;
  
  std::vector<recob::Track> const& TrackForMumCollection(*rawHandle_TPCtrackMum);   
  
  
    art::FindMany<anab::Calorimetry> trk_calo_assn_v(rawHandle_TPCtrackMum, evt, data_label_assotrackForMum_);
  
    art::Handle< std::vector<recob::MCSFitResult> > rawHandle_MCSfit;
    evt.getByLabel(data_label_MCSfit_, rawHandle_MCSfit);
    std::vector<recob::MCSFitResult> const& MCSfitCollection(*rawHandle_MCSfit);
    /*
    for (unsigned int sliceIndex = 0; sliceIndex < slices.size(); sliceIndex++)
    { // A: Loop over slices
      const float nuScore(slices.at(sliceIndex).GetTopologicalScore());
      if (nuScore > highestNuScore){
          highestNuScore = nuScore;
          mostProbableSliceIndex = sliceIndex;
      }
    }
    */
    nuScore_max = 0;//highestNuScore;

    //for (unsigned int sliceIndex = 0; sliceIndex < slices.size(); sliceIndex++)
    //{ // A: Loop over slices
      //const float nuScore(slices.at(sliceIndex).GetTopologicalScore());
      //slice_counter = sliceIndex;
      //nuScore_ = nuScore;
      //if(verbose_!=0) std::cout << "Slice " << sliceIndex << " has a nuScore of " << nuScore << std::endl;
      //PFParticleVector nu_pfparticle = slices.at(sliceIndex).GetTargetHypothesis();
      //if(verbose_!=0) std::cout << "Slice has " << nu_pfparticle.size() << " Pfparticles" << std::endl;
      
      //std::vector< art::Ptr<recob::Track> > tracks;
      //std::vector< art::Ptr<recob::Shower> > showers;
      
      //const TLorentzVector& momentumStart = nu_pfparticle.at(0).Momentum(0);
      //std::cout << "Momentum: " << momentumStart << std::endl;
      
      //crtana::auxfunc::CollectTracksAndShowers(nu_pfparticle, pfParticleHandle, evt, tracks, showers, m_trackProducerLabel, m_showerProducerLabel);
      //if(verbose_!=0) std::cout << "Of which are " << tracks.size() << " track like" << std::endl;
      //if(verbose_!=0) std::cout << "Of which are " << showers.size() << " shower like" << std::endl;
      
      //std::vector<recob::Track> const& TrackCollection_tmp(*rawHandle_TPCtrack); 
      
      
      for(std::vector<int>::size_type i = 0; i != TrackCollection.size(); i++) {    //start loop over tpc tracks
      //for(std::vector<int>::size_type i = 0; i != TrackCollection_tmp.size(); i++) {    //start loop over tpc tracks
        track_counter++;
        //reset all counters
        crthit_counter = 0;
        crttrack_counter = 0;
        crtt0_counter = 0;
        //t0_counter = 0;
        //cosmictag_counter = 0;
        //particleid_counter = 0;
        crthit_ts0 = 0;
        crttrack_ts0 = 0;
        crthit_ts1 = 0;
        crttrack_ts1 = 0;
        fAbsTimFla = 0;
        flash_PE = 0;
        flash_y = 0;
        flash_z = 0;
        calo_counter = 0;
        mcs_counter = 0;
        track_mcs_mom = 0;
        
        crthit_x = -999;
        crthit_y = -999;
        crthit_z = -999;
        crttrack_start_x = -999;
        crttrack_start_y = -999;
        crttrack_start_z = -999;
        crttrack_end_x = -999;
        crttrack_end_y = -999;
        crttrack_end_z = -999;
        
        //tpctrack_match = TrackCollection_tmp.at(i);
        
        tpctrack_match = *TrackCollection.at(i).get();
        track_length = tpctrack_match.Length();
        
        //int key = TrackCollection.at(i).key();
        //std::cout<< "Key vs loopcounter: " << key << " - " << i << std::endl;
        
        //recob::MCSFitResult fitMcs(const recob::Track& track,          int pid, bool momDepConst = true)
        
        track_start_x = tpctrack_match.Start().X();
        track_start_y = tpctrack_match.Start().Y();
        track_start_z = tpctrack_match.Start().Z();
        
        track_end_x = tpctrack_match.End().X();
        track_end_y = tpctrack_match.End().Y();
        track_end_z = tpctrack_match.End().Z();
        
        start_mom_x = tpctrack_match.MomentumVectorAtPoint(0).X();
        start_mom_y = tpctrack_match.MomentumVectorAtPoint(0).Y();
        start_mom_z = tpctrack_match.MomentumVectorAtPoint(0).Z();
        start_mom_tot = sqrt( start_mom_x*start_mom_x + start_mom_y*start_mom_y + start_mom_z*start_mom_z);
        const size_t numberTrajectoryPoints = tpctrack_match.NumberTrajectoryPoints();
        const int last = numberTrajectoryPoints - 1;
        end_mom_x = tpctrack_match.MomentumVectorAtPoint(last).X();
        end_mom_y = tpctrack_match.MomentumVectorAtPoint(last).Y();
        end_mom_z = tpctrack_match.MomentumVectorAtPoint(last).Z();
        theta_track = tpctrack_match.Theta();
        phi_track = tpctrack_match.Phi();
        zenith_angle = tpctrack_match.ZenithAngle();
        azimuth_angle = tpctrack_match.AzimuthAngle();
        
        if(verbose_!=0) std::cout << "Track properties: start=" << track_start_x << " end=" << track_end_x << " momentum:" << start_mom_x << " end_mom mom= " << end_mom_x <<std::endl;
        if(verbose_!=0) std::cout << "Track properties: theta_track=" << theta_track << " phi_track=" << phi_track << " zenith_angle:" << zenith_angle << " azimuth_angle= " << azimuth_angle << std::endl;
        
        
        //get assoziated CRT track s
        const std::vector<const crt::CRTTrack*>& CRTtrack_v = trk_crttrack_assn_v.at(TrackCollection.at(i).key());
        if(CRTtrack_v.size()>0){
          auto crttrack_tmp = CRTtrack_v.at(0);
          if(verbose_!=0){
            std::cout << "found CRTTrack assoziation" << std::endl;
            std::cout << "Track time: " << crttrack_tmp->ts0_ns << std::endl;
            std::cout << "Track position 1: " << crttrack_tmp->x1_pos << ":" << crttrack_tmp->y1_pos << ":" << crttrack_tmp->z1_pos << std::endl;
            std::cout << "Track position 2: " << crttrack_tmp->x2_pos << ":" << crttrack_tmp->y2_pos << ":" << crttrack_tmp->z2_pos << std::endl;
          }
          //update the crt track tree variables
          crttrack_ts0 = crttrack_tmp->ts0_ns;
          crttrack_ts1 = crttrack_tmp->ts1_ns;
          crttrack_start_x = crttrack_tmp->x1_pos;
          crttrack_start_y = crttrack_tmp->y1_pos;
          crttrack_start_z = crttrack_tmp->z1_pos;
          crttrack_end_x = crttrack_tmp->x2_pos;
          crttrack_end_y = crttrack_tmp->y2_pos;
          crttrack_end_z = crttrack_tmp->z2_pos;
          if(saveTTree_ >=2) crttrack_vec.push_back(*crttrack_tmp);
          crttrack_counter++;
          for(std::vector<int>::size_type j = 0; j != OpFlashCollection.size(); j++){
            auto my_flash = OpFlashCollection.at(j);
            auto Timeflash = my_flash.Time(); //in us from trigger time
            auto flash_time = fTriTim_nsec + (Timeflash * 1000);
            if( abs(flash_time - crttrack_ts0 - fCRTT0off_) < 4e3){
              fAbsTimFla = flash_time;
              flash_PE = my_flash.TotalPE();
              flash_y = my_flash.YCenter();
              flash_z = my_flash.ZCenter();
              flash_vec.push_back(my_flash);
            }
          }
            
        }
        //get assoziated crt t0 objects
        const std::vector<const crt::CRTTzero*>& CRTtzero_v = trk_crtT0_assn_v.at(TrackCollection.at(i).key());
        if(verbose_!=0) std::cout << "found CRTT0 assoziation, Size: " << CRTtzero_v.size() << std::endl;
        for(std::vector<int>::size_type j = 0; j != CRTtzero_v.size(); j++){
          if(verbose_!=0) std::cout << "found CRTT0 assoziation" << std::endl;
          auto crtT0_tmp = CRTtzero_v.at(j);
          for(std::vector<int>::size_type i_t0 = 0; i_t0 != CRTT0Collection.size(); i_t0++){
            auto my_crtt0 = CRTT0Collection.at(i_t0);
            if(saveTTree_ >=2) crtt0_vec.push_back(*crtT0_tmp);
            crtt0_counter++;
            if( (crtT0_tmp->ts0_ns == my_crtt0.ts0_ns) && (crtT0_tmp->ts0_s == my_crtt0.ts0_s) ){
              const std::vector<const crt::CRTHit*>& CRThit_v = trk_crthit_assn_v.at(i_t0);
              for(std::vector<int>::size_type i_hit = 0; i_hit != CRThit_v.size(); i_hit++){
                auto my_crthit = CRThit_v.at(i_hit);
                if(verbose_!=0){
                  std::cout << "CRT T0 time: " << crtT0_tmp->ts0_ns << std::endl;
                  std::cout << "Hit time: " << my_crthit->ts0_ns << std::endl;
                  std::cout << "Hit position: " << my_crthit->x_pos << ":" << my_crthit->y_pos << ":" << my_crthit->z_pos << std::endl;
                }
                //update the crt hit tree variables
                crthit_ts0 = my_crthit->ts0_ns;
                crthit_ts1 = my_crthit->ts1_ns;
                crthit_x = my_crthit->x_pos;
                crthit_y = my_crthit->y_pos;
                crthit_z = my_crthit->z_pos;
                if(crthit_vec.size() == 0 )  crthit_tot_counter++;
                if(saveTTree_ >=2) crthit_vec.push_back(*my_crthit);
                crthit_counter++;
               
              }
            }
          }
          int flash_found = 0;
          for(std::vector<int>::size_type j_fl = 0; j_fl != OpFlashCollection.size(); j_fl++){
            auto my_flash = OpFlashCollection.at(j_fl);
            auto Timeflash = my_flash.Time(); //in us from trigger time
            auto flash_time = fTriTim_nsec + (Timeflash * 1000);
            if( abs(Timeflash*1000 - (crthit_ts0 + fCRTT0off_ - fTriTim_nsec)) < 10000){
              if(verbose_!=0 ) std::cout << "Found flash in flashvector!" << std::endl;
              //std::cout << "CRT T0 time: " << crtT0_tmp->ts0_ns << std::endl;
              //std::cout << "Hit time - offset: " << (crthit_ts0 + fCRTT0off_ - fTriTim_nsec)/1000 << std::endl;
              //std::cout << "Hit time: " << crthit_ts0 << std::endl;
              flash_found++;
              fAbsTimFla = flash_time;
              flash_PE = my_flash.TotalPE();
              flash_y = my_flash.YCenter();
              flash_z = my_flash.ZCenter();
              if( saveTTree_>=2) flash_vec.push_back(my_flash);
              //std::cout << "Flash time: " << flash_time << std::endl;
              //std::cout << "Flash: " << Timeflash << std::endl;
              continue;
            }
          }
          if(flash_found == 0 && crthit_counter>0){
            crthit_noflash_counter++;
            if(verbose_!=0) std::cout << "Error: Not corresponding flash in flashvector found!" << std::endl;
            /*std::cout << "CRT T0 time: " << crtT0_tmp->ts0_ns << std::endl;
            std::cout << "Hit time - offset: " << (crthit_ts0 + fCRTT0off_ - fTriTim_nsec)/1000 << std::endl;
            std::cout << "Hit time: " << crthit_ts0 << std::endl;
            for(std::vector<int>::size_type j_fl = 0; j_fl != OpFlashCollection.size(); j_fl++){
              auto my_flash = OpFlashCollection.at(j_fl);
              auto Timeflash = my_flash.Time(); //in us from trigger time
              auto flash_time = fTriTim_nsec + (Timeflash * 1000);
              std::cout << "Flash time: " << flash_time << std::endl;
              std::cout << "Flash: " << Timeflash << std::endl;
              
            }
            */
          }
          if(flash_found>1){
            crthit_moreflash_counter++;
            if(verbose_!=5) std::cout << "Error: More than one flash in flashvector found!" << std::endl;
            /*
            //std::cout << "Hit time - offset: " << (crthit_ts0 + fCRTT0off_ - fTriTim_nsec)/1000 << std::endl;
            for(std::vector<int>::size_type j_fl = 0; j_fl != OpFlashCollection.size(); j_fl++){
              //auto my_flash = OpFlashCollection.at(j_fl);
              //auto Timeflash = my_flash.Time(); //in us from trigger time
              //auto flash_time = fTriTim_nsec + (Timeflash * 1000);
              //std::cout << "Flash time: " << Timeflash << std::endl;
            }
            */
          }
        }
        int pandoraTrack_found = 0;
        for(std::vector<int>::size_type i_pandoraTrack = 0; i_pandoraTrack != TrackForMumCollection.size(); i_pandoraTrack++) {    //find according pandoraTrack track for calometric info
          recob::Track my_pandoraTrack = TrackForMumCollection[i_pandoraTrack];
          
          if( abs(tpctrack_match.Start().X() - my_pandoraTrack.Start().X())<10 && abs(tpctrack_match.End().X() - my_pandoraTrack.End().X())<10){ //tpctrack_match
          if( tpctrack_match.Length() == my_pandoraTrack.Length() ){
            pandoraTrack_found++;
            if(verbose_!=0){
              std::cout << "Found corresponding pandoraTrack!" << std::endl;
              std::cout << "Track properties: mytrack=" << track_start_x << " end=" << track_end_x << " start=" << track_start_y << " end=" << track_end_y <<std::endl;
              std::cout << "Track properties: pandora track=" << my_pandoraTrack.Start().X() << " end=" << my_pandoraTrack.End().X() << " start=" << my_pandoraTrack.Start().Y() << " end=" << my_pandoraTrack.End().Y() <<std::endl;
            }
        //printf("get Calo info...\n");
        //std::cout << "Length vector asso: " << trk_calo_assn_v.size() << " Length vector MCSvec: " << MCSfitCollection.size() << " And index i= " << i << std::endl;
            const std::vector<const anab::Calorimetry*>& Calo_v = trk_calo_assn_v.at(i_pandoraTrack);
            recob::MCSFitResult my_MCSfit = MCSfitCollection[i_pandoraTrack];
            if( saveTTree_>=2) MCSfit_vec.push_back(my_MCSfit);
            track_mcs_mom = my_MCSfit.bestMomentum();
            mcs_counter++;
            //*calo_vec = Calo_v;
            calo_counter++;
            for(std::vector<int>::size_type j = 0; j < Calo_v.size(); j++){
              if( saveTTree_>=2) calo_vec.push_back(*Calo_v.at(j));
              
              if(verbose_>5){
              printf("found Calo info...\n");
              std::cout << "Track properties: startX =" << tpctrack_match.MomentumVectorAtPoint(0).X() << " eY=" << tpctrack_match.MomentumVectorAtPoint(0).Y() << " momentum Z:" << tpctrack_match.MomentumVectorAtPoint(0).Z() << " Length: " << tpctrack_match.Length() <<std::endl;
              //std::cout << "Track properties: startX =" << my_pandoraTrack.MomentumVectorAtPoint(0).X() << " eY=" << my_pandoraTrack.MomentumVectorAtPoint(0).Y() << " momentum Z:" << my_pandoraTrack.MomentumVectorAtPoint(0).Z() << " Length: " << my_pandoraTrack.Length() <<std::endl;
              std::cout << "Track properties: HasMom =" << my_MCSfit.bestMomentum() << std::endl;
              std::cout << "Track properties: Calo =" << Calo_v[j]->KineticEnergy() << std::endl;
              std::cout << "Track properties: plane =" << Calo_v[j]->PlaneID() << std::endl;
              }
            
            }
            continue;
          }
          }
        }
        
          
        
        if(pandoraTrack_found ==0  ){ 
          track_nopandora_counter++;
          
          if(verbose_!=0) std::cout << "Error: Not corresponding track in pandoraTrack found!" << std::endl;
          
          //std::cout << "Track properties: mytrack=" << track_start_x << " end=" << track_end_x << " start=" << track_start_y << " end=" << track_end_y <<std::endl;
          for(std::vector<int>::size_type i_pandoraTrack = 0; i_pandoraTrack != TrackForMumCollection.size(); i_pandoraTrack++) {    //find according pandoraTrack track for calometric info
            recob::Track my_pandoraTrack = TrackForMumCollection[i_pandoraTrack];
            //std::cout << "Track properties: pandora track=" << my_pandoraTrack.Start().X() << " end=" << my_pandoraTrack.End().X() << " start=" << my_pandoraTrack.Start().Y() << " end=" << my_pandoraTrack.End().Y() <<std::endl;
            
          }
        }
        if(pandoraTrack_found >1  ){
          if(verbose_!=0) std::cout << "Error: More than one track in pandoraTrack found!" << std::endl;
          
        }
        
        
        //fill the tree and reset the tree vectors
        //nuScore_max = highestNuScore;
        if(verbose_!=0) std::cout << "Before writing data into tree..." << std::endl;
        if(saveTTree_ != 0) my_event_->Fill();
        crthit_vec.clear();
        crttrack_vec.clear();
        crtt0_vec.clear();
        flash_vec.clear();
        calo_vec.clear();
        MCSfit_vec.clear();
        //cosmictag_vec.clear();
        //t0_vec.clear();
        //particleid_vec.clear();
      } // End loop over TPC tracks
      if(verbose_!=0) std::cout << "End loop over tracks..." << std::endl;
      
    //} // End A: Loop over slices
    if(verbose_!=0) std::cout << "End loop over slices..." << std::endl;
    //std::cout << "Tagging slice " << mostProbableSliceIndex << std::endl;

    // Tag the most probable slice as a neutrino
    //slices.at(mostProbableSliceIndex).TagAsTarget();
  
    // end tag neutrino canditates ///////////////////////////////////////


    //m_neutrinoIdTool->ClassifySlices(slices, evt);
    //PFParticleVector consolidatedParticles;
    //this->CollectConsolidatedParticles(particles, clearCosmics, slices, consolidatedParticles);

}

//------------------------------------------------------------------------------------------------------------------------------------------

void ConsolidatedOutputAnalyserAllTracks::CollectPFParticles(const art::Event &evt, PFParticleToMetadata &particlesToMetadata, PFParticleVector &particles) const
{
    if(verbose_>1) std::cout << "Enter CollectPFParticles function" << std::endl;
    art::Handle<std::vector<recob::PFParticle> > pfParticleHandle;
    evt.getByLabel(m_pandoraTag, pfParticleHandle);

    art::FindManyP<larpandoraobj::PFParticleMetadata> pfParticleMetadataAssoc(pfParticleHandle, evt, m_pandoraTag);
  
    for (unsigned int i = 0; i < pfParticleHandle->size(); ++i)
    {
        const art::Ptr<recob::PFParticle> part(pfParticleHandle, i);
        const auto &metadata(pfParticleMetadataAssoc.at(part.key()));

        particles.push_back(part);

        if (metadata.size() != 1) 
            throw cet::exception("LArPandora") << " ConsolidatedOutputAnalyserAllTracks::CollectPFParticles -- Found a PFParticle without exactly 1 metadata associated." << std::endl;

        if (!particlesToMetadata.insert(PFParticleToMetadata::value_type(part, metadata.front())).second)
            throw cet::exception("ConsolidatedOutputAnalyserAllTracks") << "Repeated PFParticles" << std::endl;
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------

void ConsolidatedOutputAnalyserAllTracks::BuildPFParticleMap(const PFParticleToMetadata &particlesToMetadata, PFParticleMap &particleMap) const
{
    if(verbose_>1) std::cout << "Enter BuildPFParticleMap function" << std::endl;
    for (const auto &entry : particlesToMetadata)
    {
        if (!particleMap.insert(PFParticleMap::value_type(entry.first->Self(), entry.first)).second)
            throw cet::exception("ConsolidatedOutputAnalyserAllTracks") << "Repeated PFParticles" << std::endl;
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------

void ConsolidatedOutputAnalyserAllTracks::CollectClearCosmicRays(const PFParticleVector &allParticles, const PFParticleToMetadata &particlesToMetadata, const PFParticleMap &particleMap, PFParticleVector &clearCosmics) const
{
    if(verbose_>1) std::cout << "Enter CollectClearCosmicRays function" << std::endl;
    for (const auto &part : allParticles)
    {
        // Get the parent of the particle
        const auto parentIt(particlesToMetadata.find(LArPandoraHelper::GetParentPFParticle(particleMap, part)));
        if (parentIt == particlesToMetadata.end())
            throw cet::exception("ConsolidatedOutputAnalyserAllTracks") << "Found PFParticle without metadata" << std::endl;

        // ATTN particles without the "IsClearCosmic" parameter are not clear cosmics
        try
        {
            if (static_cast<bool>(std::round(this->GetMetadataValue(parentIt->second, "IsClearCosmic"))))
                clearCosmics.push_back(part);
        }
        catch (const cet::exception &)
        {
        }
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------

void ConsolidatedOutputAnalyserAllTracks::CollectSlices(const PFParticleVector &allParticles, const PFParticleToMetadata &particlesToMetadata, const PFParticleMap &particleMap, SliceVector &slices) const
{
    if(verbose_>1) std::cout << "Enter CollectSlices function" << std::endl;
    std::map<unsigned int, float> nuScores;
    std::map<unsigned int, PFParticleVector> crHypotheses;
    std::map<unsigned int, PFParticleVector> nuHypotheses;

    // Collect the slice information
    for (const auto &part : allParticles)
    {
        // Find the parent PFParticle
        const auto parentIt(particlesToMetadata.find(LArPandoraHelper::GetParentPFParticle(particleMap, part)));
        if (parentIt == particlesToMetadata.end())
            throw cet::exception("ConsolidatedOutputAnalyserAllTracks") << "Found PFParticle without metadata" << std::endl;
       
        // Skip PFParticles that are clear cosmics
        try
        {
            if (static_cast<bool>(std::round(this->GetMetadataValue(parentIt->second, "IsClearCosmic"))))
                continue;
        }
        catch (const cet::exception &)
        {
        }

        const unsigned int sliceId(static_cast<unsigned int>(std::round(this->GetMetadataValue(parentIt->second, "SliceIndex"))));
        const float nuScore(this->GetMetadataValue(parentIt->second, "NuScore"));
        // ATTN all PFParticles in the same slice will have the same nuScore
        nuScores[sliceId] = nuScore;

        if (LArPandoraHelper::IsNeutrino(parentIt->first))
        {
            nuHypotheses[sliceId].push_back(part);
        }
        else 
        {
            crHypotheses[sliceId].push_back(part);
        }
    }

    // ATTN: we need to ensure that for each slice there is a cosmic and neutrino hypothesis, even if the pass created no PFOs
    // in such a case we add an empty vector of pfparticles
    const PFParticleVector emptyPFParticleVector;

    // Produce the slices
    // ATTN slice indices are enumerated from 1
    //for (unsigned int sliceId = 0; sliceId < nuScores.size(); sliceId++){
      //if(verbose_>2) std::cout << "nuScoremap: nr, key, value" << sliceId << nuScores.at(sliceId).first << nuScores.at(sliceId).second << std::endl;
    //}
  
    for (unsigned int sliceId = 1; sliceId <= nuScores.size(); sliceId++)
    {
        // Get the neutrino score
        const auto nuScoresIter(nuScores.find(sliceId));
        if (nuScoresIter == nuScores.end()){}
            //throw cet::exception("ConsolidatedOutputAnalyserAllTracks") << "Scrambled slice information - can't find nuScore with id = " << sliceId << std::endl;

        PFParticleVector nuPFParticleVector, crPFParticleVector;
        // Get the neutrino hypothesis
        const auto nuHypothesisIter(nuHypotheses.find(sliceId));
        nuPFParticleVector = ((nuHypothesisIter == nuHypotheses.end()) ? emptyPFParticleVector : nuHypothesisIter->second);

        // Get the cosmic hypothesis
        const auto crHypothesisIter(crHypotheses.find(sliceId));
        crPFParticleVector = ((crHypothesisIter == crHypotheses.end()) ? emptyPFParticleVector : crHypothesisIter->second);

        slices.emplace_back(nuScoresIter->second, nuPFParticleVector, crPFParticleVector);
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------

float ConsolidatedOutputAnalyserAllTracks::GetMetadataValue(const art::Ptr<larpandoraobj::PFParticleMetadata> &metadata, const std::string &key) const
{
    if(verbose_>1) std::cout << "Enter GetMetadataValue function" << std::endl;
    const auto &propertiesMap(metadata->GetPropertiesMap());
    const auto &it(propertiesMap.find(key));

    if (it == propertiesMap.end())
        throw cet::exception("ConsolidatedOutputAnalyserAllTracks") << "No key \"" << key << "\" found in metadata properties map" << std::endl;

    return it->second;
}

//------------------------------------------------------------------------------------------------------------------------------------------

void ConsolidatedOutputAnalyserAllTracks::CollectConsolidatedParticles(const PFParticleVector &allParticles, const PFParticleVector &clearCosmics, const SliceVector &slices, PFParticleVector &consolidatedParticles) const
{
    if(verbose_>1) std::cout << "Enter CollectConsolidatedParticles function" << std::endl;
    PFParticleVector collectedParticles;
    collectedParticles.insert(collectedParticles.end(), clearCosmics.begin(), clearCosmics.end());

    for (const auto &slice : slices)
    {
        const PFParticleVector &particles(slice.IsTaggedAsTarget() ? slice.GetTargetHypothesis() : slice.GetCosmicRayHypothesis());
        collectedParticles.insert(collectedParticles.end(), particles.begin(), particles.end());
    }

    // ATTN the collected particles are the ones we want to output, but here we loop over all particles to ensure that the consolidated 
    // particles have the same ordering.
    for (const auto &part : allParticles)
    {
        if (std::find(collectedParticles.begin(), collectedParticles.end(), part) != collectedParticles.end())
            consolidatedParticles.push_back(part);
    }
}
//------------------------------------------------------------------------------------------------------
void ConsolidatedOutputAnalyserAllTracks::initialize_tmyevent()
{
  // Implementation of optional member function here.
  std::cout << "Initialize variables and histograms for root tree output" << std::endl;
  //tree stuff for tracks: ////////////////////////////////////////////////////////////////////////////////// 
  my_event_ = tfs->make<TTree>("my_event","my_event");
  int bufsize_crthit = 128*20;
  int bufsize_flash = 152*10;
  int bufsize_crttrack = 184*5;
  int bufsize_track = 368*1;
  //int bufsize_shower = 336*1;
  //int bufsize_daqheader = 48; 
  int bufsize_crtt0 = 72*10;
  //int bufsize_t0 = 32*10;
  //int bufsize_cosmictag = 56*10;//*10?
  //int bufsize_particleid = 96*10;//*10?
  int bufsize_calorimetry = 200*3;//*10?
  int bufsize_MCSfit = 80;//*10?

  int splitlevel = 99;
  if( saveTTree_>=2) my_event_->Branch("crthits", &crthit_vec, bufsize_crthit, splitlevel);
  if( saveTTree_>=2) my_event_->Branch("crttracks", &crttrack_vec, bufsize_crttrack, splitlevel);
  if( saveTTree_>=2) my_event_->Branch("crtt0", &crtt0_vec, bufsize_crtt0, splitlevel);
  
  if( saveTTree_>=2) my_event_->Branch("flashes", &flash_vec, bufsize_flash, splitlevel);
  /*
  my_event_->Branch("cosmictag", &cosmictag_vec, bufsize_cosmictag, splitlevel);
  my_event_->Branch("t0", &t0_vec, bufsize_t0, splitlevel);
  my_event_->Branch("particleid", &particleid_vec, bufsize_particleid, splitlevel);
  */
  if( saveTTree_>=2) my_event_->Branch("tpctracks", &tpctrack_match, bufsize_track, splitlevel);
  
  if( saveTTree_>=2) my_event_->Branch("calorimetry", &calo_vec, bufsize_calorimetry, splitlevel);
  if( saveTTree_>=2) my_event_->Branch("MCSfit", &MCSfit_vec, bufsize_MCSfit, splitlevel);
  //my_event_->Branch("distance", &tpc_crthit_dist, "distance cm/D");
  my_event_->Branch("track_length", &track_length, "track_length cm/D");
  
  my_event_->Branch("track_start_x", &track_start_x, "track_start_x cm/D");
  my_event_->Branch("track_start_y", &track_start_y, "track_start_y cm/D");
  my_event_->Branch("track_start_z", &track_start_z, "track_start_z cm/D");
  my_event_->Branch("track_end_x", &track_end_x, "track_end_x cm/D");
  my_event_->Branch("track_end_y", &track_end_y, "track_end_y cm/D");
  my_event_->Branch("track_end_z", &track_end_z, "track_end_z cm/D");
  my_event_->Branch("start_mom_x", &start_mom_x, "start_mom_x cm/D");
  my_event_->Branch("start_mom_y", &start_mom_y, "start_mom_y cm/D");
  my_event_->Branch("start_mom_z", &start_mom_z, "start_mom_z cm/D");
  my_event_->Branch("start_mom_tot", &start_mom_tot, "start_mom_tot cm/D");
  my_event_->Branch("end_mom_x", &end_mom_x, "end_mom_x cm/D");
  my_event_->Branch("end_mom_y", &end_mom_y, "end_mom_y cm/D");
  my_event_->Branch("end_mom_z", &end_mom_z, "end_mom_z cm/D");
  my_event_->Branch("theta_track", &theta_track, "theta_track /D");
  my_event_->Branch("phi_track", &phi_track, "phi_track cm/D");
  my_event_->Branch("zenith_angle", &zenith_angle, "zenith_angle cm/D");
  my_event_->Branch("azimuth_angle", &azimuth_angle, "azimuth_angle cm/D");
  
  my_event_->Branch("flash_ns", &fAbsTimFla, "flash GPS ns/D");
  my_event_->Branch("flash_PE", &flash_PE, "flash_PE ns/D");
  my_event_->Branch("flash_y", &flash_y, "flash_y cm/D");
  my_event_->Branch("flash_z", &flash_z, "flash_z cm/D");
  
  my_event_->Branch("track_mcs_mom", &track_mcs_mom, "track_mcs_mom cm/D");
  
  my_event_->Branch("fTriTim_sec", &fTriTim_sec, "fTriTim_sec s/D");
  my_event_->Branch("fTriTim_nsec", &fTriTim_nsec, "fTriTim_nsec ns/D");
  
  my_event_->Branch("crttrack_ts0", &crttrack_ts0, "crttrack_ts0 ns/D");
  my_event_->Branch("crthit_ts0", &crthit_ts0, "crthit_ts0 ns/D");
  
  my_event_->Branch("crthit_x", &crthit_x, "crthit_x ns/D");
  my_event_->Branch("crthit_y", &crthit_y, "crthit_y ns/D");
  my_event_->Branch("crthit_z", &crthit_z, "crthit_z ns/D");
  
  my_event_->Branch("crttrack_start_x", &crttrack_start_x, "crttrack_start_x ns/D");
  my_event_->Branch("crttrack_start_y", &crttrack_start_y, "crttrack_start_y ns/D");
  my_event_->Branch("crttrack_start_z", &crttrack_start_z, "crttrack_start_z ns/D");
  my_event_->Branch("crttrack_end_x", &crttrack_end_x, "crttrack_end_x ns/D");
  my_event_->Branch("crttrack_end_y", &crttrack_end_y, "crttrack_end_y ns/D");
  my_event_->Branch("crttrack_end_z", &crttrack_end_z, "crttrack_end_z ns/D");
  
  my_event_->Branch("crttrack_ts1", &crttrack_ts1, "crttrack_ts1 ns/D");
  my_event_->Branch("crthit_ts1", &crthit_ts1, "crthit_ts1 ns/D");

  my_event_->Branch("nr_crthit", &crthit_counter, "nr crthit/I");
  my_event_->Branch("nr_crttrack", &crttrack_counter, "nr crttrack/I");
  my_event_->Branch("nr_crtt0", &crtt0_counter, "nr crtt0/I");
  /*
  my_event_->Branch("nr_t0", &t0_counter, "nr t0/I");
  my_event_->Branch("nr_cosmictag", &cosmictag_counter, "nr cosmictag/I");
  my_event_->Branch("nr_particleid", &particleid_counter, "nr particleid/I");
  */
  my_event_->Branch("event_nr", &event_counter, "nr event/I");
  my_event_->Branch("track_nr", &track_counter, "nr track/I");
  
  my_event_->Branch("calo_counter", &calo_counter, "nr calo/I");
  my_event_->Branch("mcs_counter", &mcs_counter, "nr mcs/I");
  
  my_event_->Branch("slice_nr", &slice_counter, "nr slice/I");
  my_event_->Branch("max_track_nr", &max_track_nr, "nr max_track/I");
  my_event_->Branch("max_shower_nr", &max_shower_nr, "nr max_shower/I");
  
  my_event_->Branch("max_slice", &max_slice, "max_slice/I");
  my_event_->Branch("nuScore", &nuScore_, "nuScore/D");
  my_event_->Branch("nuScore_max", &nuScore_max, "nuScore_max/D");

  my_event_->Branch("frunNum", &frunNum, "Run Number/i");
  my_event_->Branch("fsubRunNum", &fsubRunNum, "SubRun Number/i");
  my_event_->Branch("fEvtNum", &fEvtNum, "Event Number/i");

}
  
void ConsolidatedOutputAnalyserAllTracks::beginJob()
{
  // Implementation of optional member function here.
  //initialize_tpandora();
  initialize_tmyevent();
  //initialize_tmy_hitasso();
  //std::cout << "crthitmatch_ = " << crthitmatch_ << std::endl;

}
void ConsolidatedOutputAnalyserAllTracks::endJob()
{
  // Implementation of optional member function here.
  std::cout << "Number of tracks: " << track_counter << " Number of not assigned pandoratracks: " << track_nopandora_counter << std::endl;
  std::cout << "Number of crthits: " << crthit_tot_counter << " Number of not assigned flashes: " << crthit_noflash_counter << " Number more than one: " << crthit_moreflash_counter << std::endl;
}
} // namespace lar_pandora

