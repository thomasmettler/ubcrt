/**
 *  @file   larpandora/LArPandoraEventBuilding/NeutrinoCRTProducer_module.cc
 *
 *  @brief  module for lar pandora external event building
 */

#include "art/Framework/Core/EDProducer.h"
//#include "art/Framework/Core/EDAnalyzer.h"
#include "art/Framework/Core/ModuleMacros.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "art/Framework/Principal/Run.h"
#include "art/Framework/Principal/SubRun.h"
#include "art/Utilities/make_tool.h"

#include "canvas/Utilities/InputTag.h"
#include "canvas/Persistency/Common/FindOneP.h"
#include "fhiclcpp/ParameterSet.h"
//#include "lardata/Utilities/AssociationUtil.h"
//#include "ubcrt/CRTXSEC/CRTAnaFun.hh"

#include "messagefacility/MessageLogger/MessageLogger.h"
#include "art/Framework/Services/Optional/TFileService.h"

#include "larpandora/LArPandoraInterface/LArPandoraHelper.h"
#include "larpandora/LArPandoraEventBuilding/Slice.h"
//#include "larpandora/LArPandoraEventBuilding/NeutrinoIdBaseTool.h"
//#include "ubcrt/CRTXSEC/CRTNeutrinoIdBaseTool.h"
#include "larpandora/LArPandoraEventBuilding/LArPandoraEvent.h"

//#include "lardataobj/RecoBase/PFParticle.h"
//#include "larpandora/LArPandoraObjects/PFParticleMetadata.h"
#include "lardataobj/RecoBase/PFParticleMetadata.h"

#include "ubcrt/CRTXSEC/CRTAnaFun.hh"
//#include "ubcrt/CRTXSEC/BackTrackerTruthMatch.h"
#include "ubobj/CRT/CRTHit.hh"
#include "ubobj/CRT/CRTTrack.hh"
#include "ubobj/CRT/CRTTzero.hh"
#include "ubcrt/CRT/CRTAuxFunctions.hh"
#include "ubobj/RawData/DAQHeaderTimeUBooNE.h"

#include "lardataobj/RecoBase/Track.h"                                                                
#include "lardataobj/RecoBase/Hit.h"     
//#include "lardataobj/RecoBase/MCSFitResult.h"
//#include "larreco/RecoAlg/TrajectoryMCSFitter.h"
#include "lardataobj/AnalysisBase/T0.h" 
#include "lardataobj/AnalysisBase/ParticleID.h" 
#include "lardataobj/AnalysisBase/CosmicTag.h"                                                        
#include "lardataobj/AnalysisBase/Calorimetry.h"   
//#include "lardataobj/AnalysisBase/BackTrackerMatchingData.h" // backtracker class
//#include "lardataobj/MCBase/MCTrack.h"                                                                
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

class NeutrinoCRTProducer : public art::EDProducer
{
public:
    explicit NeutrinoCRTProducer(fhicl::ParameterSet const & pset);
    
    NeutrinoCRTProducer(NeutrinoCRTProducer const &) = delete;
    NeutrinoCRTProducer(NeutrinoCRTProducer &&) = delete;
    NeutrinoCRTProducer & operator = (NeutrinoCRTProducer const &) = delete;
    NeutrinoCRTProducer & operator = (NeutrinoCRTProducer &&) = delete;

    void produce(art::Event &evt) override;
    
    // Selected optional functions.
    void beginJob() override;
    void endJob() override;

private:
    typedef std::map<art::Ptr<recob::PFParticle>, art::Ptr<larpandoraobj::PFParticleMetadata> > PFParticleToMetadata;

    art::ServiceHandle<art::TFileService> tfs;
    TTree * my_event_;
    
    int event_counter = 0;
    uint32_t fEvtNum;                //Number of current event                       
    uint32_t frunNum;                //Run Number taken from event  
    uint32_t fsubRunNum;             //Subrun Number taken from event 
    
    double fTriTim_sec = 0;          //event trigger time sec
    double fTriTim_nsec = 0;          //event trigger time ns
    
    double fAbsTimFla = 0;
    double flash_PE = 0;
    double flash_y = 0;
    double flash_z = 0;
    int flash_inTime = -1;
    
    int nr_crthit = -1; // # crt hits assigned to a tpc track
    double crthit_ts0 = 0;
    double crthit_ts1 = 0;
    double crthit_ts0_uncorr = 0;
    int adc_length = 0;
    double crt_adc = 0;
    
    double crtt0_time = -1;
    int crtt0_trig = -1;
    double crtt0_DCA = -1;
    int crtt0_plane = -1;
    
    int has_flash = -1;
    int has_crthit = -1;
    int has_crtasso = -1;
    int has_nuslice = -1;
    
    
    double diff_flash_crt = -1;
    double diff_t0_crt = -1;
    
    void initialize_tmyevent();
    void reset_tree();
    
    void CollectPFParticles(const art::Event &evt, PFParticleToMetadata &particlesToMetadata, PFParticleVector &particles) const;
    void BuildPFParticleMap(const PFParticleToMetadata &particlesToMetadata, PFParticleMap &particleMap) const;
    void CollectClearCosmicRays(const PFParticleVector &allParticles, const PFParticleToMetadata &particlesToMetadata, const PFParticleMap &particleMap, PFParticleVector &clearCosmics) const;
    void CollectSlices(const PFParticleVector &allParticles, const PFParticleToMetadata &particlesToMetadata, const PFParticleMap &particleMap, SliceVector &slices) const;
    void CollectConsolidatedParticles(const PFParticleVector &allParticles, const PFParticleVector &clearCosmics, const SliceVector &slices, PFParticleVector &consolidatedParticles) const;
    float GetMetadataValue(const art::Ptr<larpandoraobj::PFParticleMetadata> &metadata, const std::string &key) const;

    
    int verbose_;
    int saveTTree_ = 0;
    int run_MC = 0;
    int use_ts1_ = 0;
    int fHardDelay_;
    int fCRTT0off_;
    double beam_start_ = 0;
    double beam_end_ = 0;
    
    int store_t0_ = 1;
    
    double t0_crt_window = 1.0;
    //double flash_crt_window = 1.0;

    std::string                         m_inputProducerLabel;  ///< Label for the Pandora instance that produced the collections we want to consolidated
    std::string                         data_label_DAQHeader_;
    std::string                         data_label_flash_beam_;
    std::string                         data_label_crthit_;
    std::string                         data_label_crtT0asso_;
    
    std::string                         m_trackProducerLabel;  ///< Label for the track producer using the Pandora instance that produced the collections we want to consolidate
    std::string                         m_showerProducerLabel; ///< Label for the shower producer using the Pandora instance that produced the collections we want to consolidate
    art::InputTag                       m_pandoraTag;          ///< The input tag for the pandora producer
};

DEFINE_ART_MODULE(NeutrinoCRTProducer)

} // namespace lar_pandora

//------------------------------------------------------------------------------------------------------------------------------------------
// implementation follows

#include "Pandora/PdgTable.h"

namespace lar_pandora
{

NeutrinoCRTProducer::NeutrinoCRTProducer(fhicl::ParameterSet const &pset) :
  EDProducer(pset),
    m_inputProducerLabel(pset.get<std::string>("InputProducerLabel")),
    m_trackProducerLabel(pset.get<std::string>("TrackProducerLabel")),
    m_showerProducerLabel(pset.get<std::string>("ShowerProducerLabel")),
    m_pandoraTag(art::InputTag(m_inputProducerLabel))
{
    //control variables
    verbose_ = pset.get<int>("verbose");
    saveTTree_ = pset.get<int>("saveTTree");
    run_MC = pset.get<int>("run_MC");
    //datalabels
    data_label_DAQHeader_ = pset.get<std::string>("data_label_DAQHeader");
    data_label_flash_beam_ = pset.get<std::string>("data_label_flash_beam");
    data_label_crthit_ = pset.get<std::string>("data_label_crthit");
    data_label_crtT0asso_ = pset.get<std::string>("data_label_crtT0asso");
    // crt variables
    fHardDelay_ = pset.get<int>("fHardDelay",40000);
    fCRTT0off_ = pset.get<int>("fCRTT0off",69000);
    beam_start_ = pset.get<double>("beam_start",3.2);
    beam_end_ = pset.get<double>("beam_end",5);
    use_ts1_ = pset.get<double>("int",0);
    store_t0_ = pset.get<int>("store_t0",0);
    t0_crt_window = pset.get<double>("t0_crt_window",1.0);
    
    if(store_t0_ == 1) 
    produces< std::vector<anab::T0>   >();
    
    
}

//------------------------------------------------------------------------------------------------------------------------------------------

void NeutrinoCRTProducer::produce(art::Event &evt)
{
  reset_tree();
  
  std::cout << "Prozessing event nr: " << event_counter << std::endl;
  if(verbose_!=0) std::cout << "Run " << evt.run() << ", subrun " << evt.subRun() << std::endl;
  frunNum    = evt.run();
  fsubRunNum = evt.subRun();
  fEvtNum = evt.event();
  event_counter++;
  
  PFParticleVector particles;
  PFParticleToMetadata particlesToMetadata;
  this->CollectPFParticles(evt, particlesToMetadata, particles);

  PFParticleMap particleMap;
  this->BuildPFParticleMap(particlesToMetadata, particleMap);
  
  SliceVector slices;
  this->CollectSlices(particles, particlesToMetadata, particleMap, slices);
  //std::cout << "################Her I am...#################################" << std::endl;
  //m_neutrinoIdTool->ClassifySlicesCRTHit(slices, evt, m_crthitLabel);
  // tag neutrino candidates ///////////////////////////////////////////
  int max_slice = slices.size();
  if(verbose_!=0) std::cout << "Number of Slices: " << max_slice << std::endl;
  
  std::vector< art::Ptr<recob::Track> > TrackCollection;
  
  art::Handle<std::vector<recob::PFParticle> > pfParticleHandle;
  evt.getByLabel(m_inputProducerLabel, pfParticleHandle);
  
  art::Handle<std::vector<recob::Track> > rawHandle_TPCtrack;
  evt.getByLabel(m_pandoraTag, rawHandle_TPCtrack);
  art::FindMany<anab::T0> trk_T0_assn_v(rawHandle_TPCtrack, evt, data_label_crtT0asso_);
  
  for(unsigned int sliceIndex = 0; sliceIndex < slices.size(); sliceIndex++){
    //if( slices.at(sliceIndex).IsTaggedAsTarget () ){
    PFParticleVector nu_pfparticle;
    nu_pfparticle = slices.at(sliceIndex).GetTargetHypothesis();
    PFParticleVector outputParticles;
    lar_pandora::LArPandoraHelper::SelectNeutrinoPFParticles	(	nu_pfparticle, outputParticles );
    if(verbose_!=0) std::cout << "Number of neutrinos in neutrino slice: " << outputParticles.size() << std::endl;
    if(outputParticles.size()!=0){
      has_nuslice = 1;
      std::vector< art::Ptr<recob::Track> > tracks_slice;
      std::vector< art::Ptr<recob::Shower> > showers_slice;
      crtana::auxfunc::CollectTracksAndShowers(nu_pfparticle, pfParticleHandle, evt, tracks_slice, showers_slice, m_trackProducerLabel, m_showerProducerLabel);
      int max_track_nr = tracks_slice.size();
      int max_shower_nr = showers_slice.size();
      if(verbose_ !=0 )std::cout << "Track and Shower multiplicity: " << max_track_nr  << " - " << max_shower_nr << std::endl;

      TrackCollection.insert(TrackCollection.end(), tracks_slice.begin(), tracks_slice.end());
    }
    
  }
  if(verbose_!=0) std::cout << "Number of tracks in neutrino slice: " << TrackCollection.size() << std::endl;
  
  
  for(std::vector<int>::size_type i = 0; i != TrackCollection.size(); i++) {    //start loop over tpc tracks
    const std::vector<const anab::T0*>& T0crt = trk_T0_assn_v.at(TrackCollection.at(i).key());
    if(T0crt.size()!=0){
      if(verbose_!=0){
        std::cout << "################################################" << std::endl;
        std::cout << "Found T0 object from crthit - track association:" << std::endl;
        std::cout << "## Time [us]:\t\t" << T0crt.at(0)->Time() << std::endl;
        std::cout << "## Is from CRT []:\t" << T0crt.at(0)->TriggerType() << std::endl;
        std::cout << "## DCA [cm]:\t\t" << T0crt.at(0)->TriggerConfidence() << std::endl;
        std::cout << "## plane:\t\t" << T0crt.at(0)->TriggerBits() << std::endl;
        std::cout << "################################################" << std::endl;
      }
      //fill tree variables
      crtt0_time = T0crt.at(0)->Time();
      crtt0_trig = T0crt.at(0)->TriggerType();
      crtt0_DCA = T0crt.at(0)->TriggerConfidence();
      crtt0_plane = T0crt.at(0)->TriggerBits();
      has_crtasso = 1;
    }
  }
  
  int beam_flash_index = -1;
  int beam_crthit_index = -1;
  
  art::Handle< std::vector<recob::OpFlash> > rawHandle_OpFlashBeam;
  evt.getByLabel(data_label_flash_beam_, rawHandle_OpFlashBeam);
  std::vector<recob::OpFlash> const& OpFlashCollectionBeam(*rawHandle_OpFlashBeam);
  
  if(verbose_!=0) std::cout << "There are: " << OpFlashCollectionBeam.size() << " in the beamflash collection" << std::endl;
  
  
  for(std::vector<int>::size_type i = 0; i != OpFlashCollectionBeam.size(); i++) {
    /*std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;
    std::cout << "Flash in beam window:" << std::endl;
    std::cout << "## Time [us]:\t\t" << OpFlashCollectionBeam.at(i).Time() << std::endl;
    std::cout << "## Total PE []:\t" << OpFlashCollectionBeam.at(i).TotalPE() << std::endl;
    std::cout << "## Y [cm]:\t\t" << OpFlashCollectionBeam.at(i).YCenter() << std::endl;
    std::cout << "## Z [cm]:\t\t" << OpFlashCollectionBeam.at(i).ZCenter() << std::endl;
    std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl;*/
    
    //if( OpFlashCollectionBeam.at(i).InBeamFrame()){ //abs( crthittime - 4.1) < 0.9
    if( abs( OpFlashCollectionBeam.at(i).Time() - (beam_end_ + beam_start_)/2 ) < (beam_end_-beam_start_)/2 ){
      if(verbose_!=0){
      std::cout << "################################################" << std::endl;
      std::cout << "Flash in beam window:" << std::endl;
      std::cout << "## Time [us]:\t\t" << OpFlashCollectionBeam.at(i).Time() << std::endl;
      std::cout << "## Total PE []:\t" << OpFlashCollectionBeam.at(i).TotalPE() << std::endl;
      std::cout << "## Y [cm]:\t\t" << OpFlashCollectionBeam.at(i).YCenter() << std::endl;
      std::cout << "## Z [cm]:\t\t" << OpFlashCollectionBeam.at(i).ZCenter() << std::endl;
      std::cout << "################################################" << std::endl;
      }
      //fill tree variables
      fAbsTimFla = OpFlashCollectionBeam.at(i).Time();
      flash_PE = OpFlashCollectionBeam.at(i).TotalPE();
      flash_y = OpFlashCollectionBeam.at(i).YCenter();
      flash_z = OpFlashCollectionBeam.at(i).ZCenter();
      flash_inTime = 1;
      has_flash = 1;
      beam_flash_index = i;
    }
    
  }
  
  art::Handle< raw::DAQHeaderTimeUBooNE > rawHandle_DAQHeader;
  evt.getByLabel(data_label_DAQHeader_, rawHandle_DAQHeader);
  raw::DAQHeaderTimeUBooNE const& my_DAQHeader(*rawHandle_DAQHeader);
  art::Timestamp evtTimeGPS = my_DAQHeader.gps_time();  
  fTriTim_sec = evtTimeGPS.timeHigh();
  fTriTim_nsec = evtTimeGPS.timeLow();
  
  art::Handle< std::vector<crt::CRTHit> > rawHandle_CRTHit;
  evt.getByLabel(data_label_crthit_, rawHandle_CRTHit); //mergerextra
  std::vector<crt::CRTHit> const& CRTHitCollection(*rawHandle_CRTHit);
  
  double crtts0_match = 0; //mark if running on MC match is found with ts0 or ts1
  
  for(std::vector<int>::size_type i = 0; i != CRTHitCollection.size(); i++) {
    if(run_MC == 1){
      double crthittime = (CRTHitCollection.at(i).ts1_ns);
      crthittime = crthittime/1000.0;
      // try bnb matching with ts1 first
      if( abs( crthittime - (beam_end_ + beam_start_)/2 ) < (beam_end_-beam_start_)/2 ){
        crthit_ts1 = ((double)CRTHitCollection.at(i).ts1_ns)/1000.0;
        if(verbose_!=0){
          std::cout << "################################################" << std::endl;
          std::cout << "CRT Hit in beam window:" << std::endl;
          std::cout << "## Time GPS [us]:\t" << crthittime << std::endl;
          std::cout << "## Time Beam [us]:\t" << crthit_ts1 << std::endl;
          std::cout << "################################################" << std::endl;
        }
        //fill tree variables
        crthit_ts0_uncorr = (double)(CRTHitCollection.at(i).ts0_ns + fCRTT0off_ - fTriTim_nsec)/1000.0;
        crthit_ts1 = ((double)CRTHitCollection.at(i).ts1_ns)/1000.0;
        crthit_ts0 = crthittime;
        adc_length = CRTHitCollection.at(i).pesmap.begin()->second.size();
        if(verbose_!=0) std::cout << "## length of adc vector:\t" << adc_length << std::endl;
        crt_adc = CRTHitCollection.at(i).peshit;
        nr_crthit++;
        has_crthit = 1;
        crtts0_match = 0;
        beam_crthit_index = i;
      }
      else{ //use ts0 for bnb window test
        double crthittime = (CRTHitCollection.at(i).ts0_ns + fCRTT0off_ - fTriTim_nsec);
        crthittime = crthittime/1000.0;
        if( abs( crthittime - (beam_end_ + beam_start_)/2 ) < (beam_end_-beam_start_)/2 ){
          if(verbose_!=0){
            std::cout << "################################################" << std::endl;
            std::cout << "CRT Hit in beam window:" << std::endl;
            std::cout << "## Time GPS [us]:\t" << crthittime << std::endl;
            std::cout << "## Time Beam [us]:\t" << crthit_ts1 << std::endl;
            std::cout << "################################################" << std::endl;
          }
          //fill tree variables
          crthit_ts0_uncorr = (double)(CRTHitCollection.at(i).ts0_ns + fCRTT0off_ - fTriTim_nsec)/1000.0;
          crthit_ts1 = ((double)CRTHitCollection.at(i).ts1_ns)/1000.0;
          crthit_ts0 = crthittime;
          adc_length = CRTHitCollection.at(i).pesmap.begin()->second.size();
          if(verbose_!=0) std::cout << "## length of adc vector:\t" << adc_length << std::endl;
          crt_adc = CRTHitCollection.at(i).peshit;
          nr_crthit++;
          has_crthit = 1;
          crtts0_match = 1;
          beam_crthit_index = i;
        }
      }
    }
    else{
      double crthittime = 0;
      if(use_ts1_ == 1) crthittime = (CRTHitCollection.at(i).ts1_ns + fHardDelay_);
      else crthittime = (CRTHitCollection.at(i).ts0_ns + fCRTT0off_ - fTriTim_nsec);
      
      crthittime = crthittime/1000.0;

      if( abs( crthittime - (beam_end_ + beam_start_)/2 ) < (beam_end_-beam_start_)/2 ){
        if(verbose_!=0){
          std::cout << "################################################" << std::endl;
          std::cout << "CRT Hit in beam window:" << std::endl;
          std::cout << "## Time GPS [us]:\t" << crthittime << std::endl;
          std::cout << "## Time Beam [us]:\t" << crthit_ts1 << std::endl;
          std::cout << "################################################" << std::endl;
        }
        //fill tree variables
        crthit_ts0 = (double)(CRTHitCollection.at(i).ts0_ns + fCRTT0off_ - fTriTim_nsec)/1000.0;
        crthit_ts1 = ((double)CRTHitCollection.at(i).ts1_ns + fHardDelay_)/1000.0;
        nr_crthit++;
        has_crthit = 1;
        beam_crthit_index = i;
      }
    }
    
  }
  if(verbose_!=0){
    std::cout << "################################################" << std::endl;
    std::cout << "Flash in beam: " << beam_flash_index << " CRT hit in beam: " << beam_crthit_index << std::endl;
    std::cout << "################################################" << std::endl;
  }
  
  if(beam_flash_index!=-1 && beam_crthit_index!=-1){
    double flash_time = OpFlashCollectionBeam.at(beam_flash_index).Time();
    double crt_time = 0;
    if(use_ts1_ == 1) crt_time = (CRTHitCollection.at(beam_crthit_index).ts1_ns + fHardDelay_);
    else crt_time = (CRTHitCollection.at(beam_crthit_index).ts0_ns + fCRTT0off_ - fTriTim_nsec);
    crt_time = crt_time/1000.0;
    if(run_MC == 1){
      crt_time = (double)(CRTHitCollection.at(beam_crthit_index).ts1_ns)/1000.0;
      if(crtts0_match == 1){
        crt_time = (CRTHitCollection.at(beam_crthit_index).ts0_ns + fCRTT0off_ - fTriTim_nsec);
      }
    }
    double time_diff = flash_time - crt_time;
    if(verbose_!=0){
      std::cout << "################################################" << std::endl;
      std::cout << "Flash - CRThit time difference: " << time_diff << std::endl;
      std::cout << "################################################" << std::endl;
    }
    diff_flash_crt = time_diff;
    
    if( abs(crtt0_time - crt_time) < t0_crt_window ){
      if(verbose_!=0){
        std::cout << "################################################" << std::endl;
        std::cout << "CRT Hit is assoziated to slice!" << std::endl;
        std::cout << "################################################" << std::endl;
      }
      
       diff_t0_crt = crtt0_time - crt_time;
    }
  }
  
  if(saveTTree_ != 0) my_event_->Fill();
  
  // produce anab::t0 object to tag event
  std::unique_ptr<std::vector<anab::T0> > T0_collection(new std::vector<anab::T0>);
  if(has_nuslice){
    anab::T0 my_t0;
    my_t0.fTime = crthit_ts0;  // double
    my_t0.fTriggerType = has_flash;    //unsigned int
    my_t0.fTriggerBits = nr_crthit;    //int
    my_t0.fID = has_crtasso;             //int
    my_t0.fTriggerConfidence = diff_flash_crt; //double
    T0_collection->emplace_back(my_t0);
  }
  if(store_t0_ == 1)
    evt.put(std::move(T0_collection));
  
}
void NeutrinoCRTProducer::CollectPFParticles(const art::Event &evt, PFParticleToMetadata &particlesToMetadata, PFParticleVector &particles) const
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
            throw cet::exception("LArPandora") << " NeutrinoCRTProducer::CollectPFParticles -- Found a PFParticle without exactly 1 metadata associated." << std::endl;

        if (!particlesToMetadata.insert(PFParticleToMetadata::value_type(part, metadata.front())).second)
            throw cet::exception("NeutrinoCRTProducer") << "Repeated PFParticles" << std::endl;
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------

void NeutrinoCRTProducer::BuildPFParticleMap(const PFParticleToMetadata &particlesToMetadata, PFParticleMap &particleMap) const
{
    if(verbose_>1) std::cout << "Enter BuildPFParticleMap function" << std::endl;
    for (const auto &entry : particlesToMetadata)
    {
        if (!particleMap.insert(PFParticleMap::value_type(entry.first->Self(), entry.first)).second)
            throw cet::exception("NeutrinoCRTProducer") << "Repeated PFParticles" << std::endl;
    }
}

//------------------------------------------------------------------------------------------------------------------------------------------

void NeutrinoCRTProducer::CollectClearCosmicRays(const PFParticleVector &allParticles, const PFParticleToMetadata &particlesToMetadata, const PFParticleMap &particleMap, PFParticleVector &clearCosmics) const
{
    if(verbose_>1) std::cout << "Enter CollectClearCosmicRays function" << std::endl;
    for (const auto &part : allParticles)
    {
        // Get the parent of the particle
        const auto parentIt(particlesToMetadata.find(LArPandoraHelper::GetParentPFParticle(particleMap, part)));
        if (parentIt == particlesToMetadata.end())
            throw cet::exception("NeutrinoCRTProducer") << "Found PFParticle without metadata" << std::endl;

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

void NeutrinoCRTProducer::CollectSlices(const PFParticleVector &allParticles, const PFParticleToMetadata &particlesToMetadata, const PFParticleMap &particleMap, SliceVector &slices) const
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
            throw cet::exception("ConsolidatedOutputAnalyserSlices_module") << "Found PFParticle without metadata" << std::endl;
       
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
  
    for (unsigned int sliceId = 1; sliceId <= nuScores.size(); sliceId++)
    {
        // Get the neutrino score
        const auto nuScoresIter(nuScores.find(sliceId));
        if (nuScoresIter == nuScores.end()){}
            //throw cet::exception("NeutrinoCRTProducer") << "Scrambled slice information - can't find nuScore with id = " << sliceId << std::endl;

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

float NeutrinoCRTProducer::GetMetadataValue(const art::Ptr<larpandoraobj::PFParticleMetadata> &metadata, const std::string &key) const
{
    if(verbose_>1) std::cout << "Enter GetMetadataValue function" << std::endl;
    const auto &propertiesMap(metadata->GetPropertiesMap());
    const auto &it(propertiesMap.find(key));

    if (it == propertiesMap.end())
        throw cet::exception("NeutrinoCRTProducer") << "No key \"" << key << "\" found in metadata properties map" << std::endl;

    return it->second;
}

//-----------------------------art::Assns<recob::Track,anab::T0,void>-------------------------------------------------------------------------------------------------------------

void NeutrinoCRTProducer::CollectConsolidatedParticles(const PFParticleVector &allParticles, const PFParticleVector &clearCosmics, const SliceVector &slices, PFParticleVector &consolidatedParticles) const
{
    if(verbose_>1) std::cout << "Enter CollectConsolidatedParticles function" << std::endl;
    PFParticleVector collectedParticles;
    collectedParticles.insert(collectedParticles.end(), clearCosmics.begin(), clearCosmics.end());

    for (const auto &slice : slices)
    {
        const PFParticleVector &particles(slice.IsTaggedAsTarget() ? slice.GetTargetHypothesis() : slice.GetCosmicRayHypothesis());
        collectedParticles.insert(collectedParticles.end(), particles.begin(), particles.end());
    }

    // ATTN the collected partiart::Assns<recob::Track,anab::T0,void>cles are the ones we want to output, but here we loop over all particles to ensure that the consolidated 
    // particles have the same ordering.
    for (const auto &part : allParticles)
    {
        if (std::find(collectedParticles.begin(), collectedParticles.end(), part) != collectedParticles.end())
            consolidatedParticles.push_back(part);
    }
}

//------------------------------------------------------------------------------------------------------
void NeutrinoCRTProducer::initialize_tmyevent()
{
  // Implementation of optional member function here.
  std::cout << "Initialize variables and histograms for root tree output" << std::endl;
  
  // tree stuff for tracks: ////////////////////////////////////////////////////////////////////////////////// 
  my_event_ = tfs->make<TTree>("my_event","my_event");
  // event infos
  my_event_->Branch("run_MC", &run_MC, "run_MC/I");
  my_event_->Branch("event_counter", &event_counter, "event_counter/I");
  my_event_->Branch("frunNum", &frunNum, "Run Number/i");
  my_event_->Branch("fsubRunNum", &fsubRunNum, "SubRun Number/i");
  my_event_->Branch("fEvtNum", &fEvtNum, "Event Number/i");
  // DAQ time info
  my_event_->Branch("fTriTim_sec", &fTriTim_sec, "fTriTim_sec s/D");
  my_event_->Branch("fTriTim_nsec", &fTriTim_nsec, "fTriTim_nsec ns/D");
  // Beam flash info
  my_event_->Branch("flash_time", &fAbsTimFla, "flash time us/D");
  my_event_->Branch("flash_PE", &flash_PE, "flash_PE ns/D");
  my_event_->Branch("flash_y", &flash_y, "flash_y cm/D");
  my_event_->Branch("flash_z", &flash_z, "flash_z cm/D");
  my_event_->Branch("flash_inTime", &flash_inTime, "is in BNB/I");
  // crt hit info
  my_event_->Branch("nr_crthit", &nr_crthit, "nr_crthit in BNB/I");
  my_event_->Branch("crthit_ts0", &crthit_ts0, "crthit_ts0 ns/D");
  my_event_->Branch("crthit_ts1", &crthit_ts1, "crthit_ts1 ns/D");
  my_event_->Branch("crthit_ts0_uncorr", &crthit_ts0_uncorr, "crthit_ts0_uncorr ns/D");
  my_event_->Branch("adc_length", &adc_length, "adc_length/I");
  my_event_->Branch("peshit", &crt_adc, "peshit/D");
  // asso tcrt t0 
  my_event_->Branch("crtt0_time", &crtt0_time, "crtt0_time us/D");
  my_event_->Branch("crtt0_trig", &crtt0_trig, "crtt0_trig = 2/I");
  my_event_->Branch("crtt0_DCA", &crtt0_DCA, "crtt0_DCA ncm/D");
  my_event_->Branch("crtt0_plane", &crtt0_plane, "crtt0_plane 3=top/I");
  //general event info
  my_event_->Branch("has_flash", &has_flash, "has_flash in beam/I");
  my_event_->Branch("has_crthit", &has_crthit, "has_crthit in beam/I");
  my_event_->Branch("has_crtasso", &has_crtasso, "has_crtasso in slice/I");
  my_event_->Branch("has_nuslice", &has_nuslice, "has_nuslice/I");
  
  my_event_->Branch("diff_flash_crt", &diff_flash_crt, "diff_flash_crt us/D");
  my_event_->Branch("diff_t0_crt", &diff_t0_crt, "diff_t0_crt us/D");
  
  
  
}

void NeutrinoCRTProducer::reset_tree()
{
  fAbsTimFla = -1;
  flash_PE = -1;
  flash_y = -1;
  flash_z = -1;
  flash_inTime = 0;
  nr_crthit = 0;
  crthit_ts0 = -1;
  crthit_ts1 = -1;
  crthit_ts0_uncorr = -1;
  adc_length = -1;
  crt_adc = -1;
  crtt0_time = -1;
  crtt0_trig = -1;
  crtt0_DCA = -1;
  crtt0_plane = -1;
  has_flash = 0;
  has_crthit = 0;
  has_crtasso = 0;
  has_nuslice = 0;
  diff_flash_crt = -1;
  diff_t0_crt = -1;
}
  
  
void NeutrinoCRTProducer::beginJob()
{
  // Implementation of optional member function here.
  std::cout << "Starting Neutrino CRT producer module" << std::endl;
  //initialize_tpandora();
  initialize_tmyevent();
  std::cout << "-------Using the following fcl parameters:-------" << std::endl;
  std::cout << "Pandora label:\t\t" << m_inputProducerLabel << std::endl;
  //std::cout << "Track label:\t\t" << m_inputProducerLabel << std::endl;
  std::cout << "CRT T0 asso label:\t" << data_label_crtT0asso_ << std::endl;
  std::cout << "Beam flash label:\t" << data_label_flash_beam_ << std::endl;
  std::cout << "CRT hit label:\t\t" << data_label_crthit_ << std::endl;
  
  std::cout << "fHardDelay:\t\t" << fHardDelay_ << std::endl;
  std::cout << "fCRTT0off:\t\t" << fCRTT0off_ << std::endl;
  std::cout << "Beam start:\t\t" << beam_start_ << std::endl;
  std::cout << "Beam end:\t\t" << beam_end_ << std::endl;
  std::cout << "Use ts1:\t\t" << use_ts1_ << std::endl;
  std::cout << "saveTTree:\t\t" << saveTTree_ << std::endl;
  std::cout << "run_MC:\t\t\t" << run_MC << std::endl;
  std::cout << "verbose:\t\t\t" << verbose_ << std::endl;
  std::cout << "store_t0:\t\t\t" << store_t0_ << std::endl;
  std::cout << "t0_crt_window:\t\t\t" << t0_crt_window << std::endl;
  std::cout << "------------end fcl parameters-------------------" << std::endl;
  //initialize_tmy_hitasso();
  //std::cout << "crthitmatch_ = " << crthitmatch_ << std::endl;

}
void NeutrinoCRTProducer::endJob()
{
  // Implementation of optional member function here.
  //std::cout << "Number of tracks: " << track_counter << " Number of not assigned pandoratracks: " << track_nopandora_counter << std::endl;
 // std::cout << "Number of crthits: " << crthit_tot_counter << " Number of not assigned flashes: " << crthit_noflash_counter << " Number more than one: " << crthit_moreflash_counter << std::endl;
}
} // namespace lar_pandora

