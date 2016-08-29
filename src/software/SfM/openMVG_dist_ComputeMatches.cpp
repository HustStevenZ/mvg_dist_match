// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/pipelines/sfm_engine.hpp"
#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"

/// Generic Image Collection image matching
#include "openMVG/matching_image_collection/Matcher_Regions_AllInMemory.hpp"
#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions_AllInMemory.hpp"
#include "openMVG/matching_image_collection/GeometricFilter.hpp"
#include "openMVG/matching_image_collection/F_ACRobust.hpp"
#include "openMVG/matching_image_collection/E_ACRobust.hpp"
#include "openMVG/matching_image_collection/H_ACRobust.hpp"
#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp"
#include "openMVG/matching/indMatch_utils.hpp"
#include "openMVG/matching/dist_match_cmd.hpp"
#include "openMVG/system/timer.hpp"

#include "openMVG/graph/graph.hpp"
#include "openMVG/stl/stl.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include "cpp_redis/cpp_redis"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <cereal/archives/json.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>
#include <iostream>
#include <zmqpp/zmqpp.hpp>

#include <cstdlib>
#include <fstream>

#include <mutex>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::matching;
using namespace openMVG::robust;
using namespace openMVG::sfm;
using namespace openMVG::matching_image_collection;
using namespace std;

struct ClientConfig{
    std::string serverIp;
    std::string redisServerIp;
    std::string localIp;
    int redisServerPort;
    template <typename Archive>
            void serialize(Archive& ar)
    {
        ar(cereal::make_nvp("serverIp",serverIp),
        cereal::make_nvp("redisServerIp",redisServerIp),
        cereal::make_nvp("redisServerPort",redisServerPort),
        cereal::make_nvp("localIp",localIp));
    }
};

struct ServerConfig{

    std::vector<std::string> client_vec;
    std::string redisServerIp;
    int redisServerPort;
    int packageSize;
    template <typename Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::make_nvp("clients",client_vec),
           cereal::make_nvp("redisServerIp",redisServerIp),
           cereal::make_nvp("redisServerPort",redisServerPort),
        cereal::make_nvp("packageSize",packageSize));
    }
};

int global_next_task = 0;//Next task index
std::map<std::string,std::string> global_workerMap;
std::map<std::string,zmqpp::socket*> global_worker_sockets;
Pair_Vec pair_vec;
std::mutex global_worker_mutex;
PairWiseMatches global_result_map;
std::mutex global_result_mutex;


std::shared_ptr<Regions_Provider> regions_provider = std::make_shared<Regions_Provider>();
std::unique_ptr<openMVG::features::Regions> regions_type(nullptr);

cpp_redis::redis_client redisConnnector;

volatile std::atomic_bool should_redis_replied(false);
volatile std::atomic_int should_all_clients_synced(0);

const int global_client_listen_port = 5678;
const int global_client_send_port = 5679;
const int global_server_listen_port = 8765;
const int global_server_send_port = 8764;

const int global_fix_task_pack_size = 10;

zmqpp::context context;
zmqpp::socket* sendsocket;
zmqpp::socket* recvsocket;

zmqpp::poller* poller;
//zmqpp::reactor* reactor;

void initWorker(ServerConfig & serverConfig)
{
    for(auto& workerId:serverConfig.client_vec) {
        global_workerMap[workerId] = "idle";
        global_result_map.clear();

        zmqpp::socket *socket = new zmqpp::socket(context, zmqpp::socket_type::push);
        std::stringstream connecturl;
        connecturl << "tcp://" << workerId << ":" << global_client_listen_port;
        socket->connect(connecturl.str());

        global_worker_sockets[workerId] = socket;
    }
}

void addFinished_Result(const PairWiseMatches& results)
{

    global_result_mutex.lock();

//    for(auto& key: results.key_comp())
    for(auto& kv:results)
    {
        global_result_map.insert(kv);
    }
    global_result_mutex.unlock();
}

bool isWorkerIdle(string workerId)
{
    return global_workerMap[workerId] == "idle";
}

void setWorkerBusy(string workerId)
{
    global_worker_mutex.lock();
    global_workerMap[workerId] = "busy";
    global_worker_mutex.unlock();
}

void setWorkerIdle(string workerId)
{

    global_worker_mutex.lock();
    global_workerMap[workerId] = "idle";
    global_worker_mutex.unlock();
}

enum EGeometricModel
{
    FUNDAMENTAL_MATRIX = 0,
    ESSENTIAL_MATRIX   = 1,
    HOMOGRAPHY_MATRIX  = 2
};

enum EPairMode
{
    PAIR_EXHAUSTIVE = 0,
    PAIR_CONTIGUOUS = 1,
    PAIR_FROM_FILE  = 2
};

void handleWaitForSync()
{
    while(should_all_clients_synced<global_worker_sockets.size())
    {
        string data;
        recvsocket->receive(data);
        std::istringstream i_archive_stream( data );
        FinishTaskCommand finishTaskCommand;
        {
            cereal::JSONInputArchive ar(i_archive_stream);
            ar(finishTaskCommand);
        }
        if(finishTaskCommand.matches.size()==0)
        {
            should_all_clients_synced++;
        }
    }
}

void handleServerRecvFinish()
{
    while(global_result_map.size()<pair_vec.size())
    {

        string data;
        recvsocket->receive(data);
        std::istringstream i_archive_stream( data );
        FinishTaskCommand finishTaskCommand;
        {
            cereal::JSONInputArchive ar(i_archive_stream);
            ar(finishTaskCommand);
        }
        std::cout<<"Server recved result form client "<<finishTaskCommand.workerId<<std::endl;
        addFinished_Result(finishTaskCommand.matches);
        setWorkerIdle(finishTaskCommand.workerId);
    }

}

/// Compute corresponding features between a series of views:
/// - Load view images description (regions: features & descriptors)
/// - Compute putative local feature matches (descriptors matching)
/// - Compute geometric coherent feature matches (robust model estimation from putative matches)
/// - Export computed data
int main(int argc, char **argv)
{
    CmdLine cmd;

    std::string sSfM_Data_Filename;
    std::string sMatchesDirectory = "";
    std::string sGeometricModel = "f";
    float fDistRatio = 0.8f;
    int iMatchingVideoMode = -1;
    std::string sPredefinedPairList = "";
    bool bUpRight = false;
    std::string sNearestMatchingMethod = "AUTO";
    bool bForce = false;
    bool bGuided_matching = false;
    int imax_iteration = 2048;
    bool bDistributed_matching = false;
    bool bIsDistributedServer = false;
    std::string sServerIp = "127.0.0.1";
    std::string sConfigFile = "";
    //required
    cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
    cmd.add( make_option('o', sMatchesDirectory, "out_dir") );
    // Options
    cmd.add( make_option('r', fDistRatio, "ratio") );
    cmd.add( make_option('g', sGeometricModel, "geometric_model") );
    cmd.add( make_option('v', iMatchingVideoMode, "video_mode_matching") );
    cmd.add( make_option('l', sPredefinedPairList, "pair_list") );
    cmd.add( make_option('n', sNearestMatchingMethod, "nearest_matching_method") );
    cmd.add( make_option('f', bForce, "force") );
    cmd.add( make_option('m', bGuided_matching, "guided_matching") );
    cmd.add( make_option('I', imax_iteration, "max_iteration") );
    cmd.add( make_option('D',bDistributed_matching,"isDistributed"));
    cmd.add( make_option('S',bIsDistributedServer,"isDistributedServer"));
    cmd.add( make_option('s',sServerIp,"sServerIp"));
    cmd.add( make_option('c',sConfigFile,"sConfigFile"));

    try {
        if (argc == 1) throw std::string("Invalid command line parameter.");
        cmd.process(argc, argv);
    } catch(const std::string& s) {
        std::cerr << "Usage: " << argv[0] << '\n'
        << "[-i|--input_file] a SfM_Data file\n"
        << "[-o|--out_dir path] output path where computed are stored\n"
        << "\n[Optional]\n"
        << "[-f|--force] Force to recompute data]\n"
        << "[-r|--ratio] Distance ratio to discard non meaningful matches\n"
        << "   0.8: (default).\n"
        << "[-g|--geometric_model]\n"
        << "  (pairwise correspondences filtering thanks to robust model estimation):\n"
        << "   f: (default) fundamental matrix,\n"
        << "   e: essential matrix,\n"
        << "   h: homography matrix.\n"
        << "[-v|--video_mode_matching]\n"
        << "  (sequence matching with an overlap of X images)\n"
        << "   X: with match 0 with (1->X), ...]\n"
        << "   2: will match 0 with (1,2), 1 with (2,3), ...\n"
        << "   3: will match 0 with (1,2,3), 1 with (2,3,4), ...\n"
        << "[-l]--pair_list] file\n"
        << "[-n|--nearest_matching_method]\n"
        << "  AUTO: auto choice from regions type,\n"
        << "  For Scalar based regions descriptor:\n"
        << "    BRUTEFORCEL2: L2 BruteForce matching,\n"
        << "    ANNL2: L2 Approximate Nearest Neighbor matching,\n"
        << "    CASCADEHASHINGL2: L2 Cascade Hashing matching.\n"
        << "    FASTCASCADEHASHINGL2: (default)\n"
        << "      L2 Cascade Hashing with precomputed hashed regions\n"
        << "     (faster than CASCADEHASHINGL2 but use more memory).\n"
        << "  For Binary based descriptor:\n"
        << "    BRUTEFORCEHAMMING: BruteForce Hamming matching.\n"
        << "[-m|--guided_matching]\n"
        << "[-D] true for enabling distributed computing\n"
        << "[-S] true for run as distributed server\n"
        << "  use the found model to improve the pairwise correspondences."
        << std::endl;

        std::cerr << s << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << " You called : " << "\n"
    << argv[0] << "\n"
    << "--input_file " << sSfM_Data_Filename << "\n"
    << "--out_dir " << sMatchesDirectory << "\n"
    << "Optional parameters:" << "\n"
    << "--force " << bForce << "\n"
    << "--ratio " << fDistRatio << "\n"
    << "--geometric_model " << sGeometricModel << "\n"
    << "--video_mode_matching " << iMatchingVideoMode << "\n"
    << "--pair_list " << sPredefinedPairList << "\n"
    << "--nearest_matching_method " << sNearestMatchingMethod << "\n"
    << "--isDistributed"<< bDistributed_matching<<"\n"
    << "--guided_matching " << bGuided_matching << std::endl;



    ServerConfig serverConfig;
    ClientConfig clientConfig;
    if(bDistributed_matching && bIsDistributedServer)
    {
        std::ifstream configFile(sConfigFile);
        if(configFile.is_open())
        {
            cereal::JSONInputArchive ar(configFile);
            ar(serverConfig);
        }
        else
        {
            std::ofstream serverf("serverconfig.json");
            if(serverf.is_open())
            {
                cereal::JSONOutputArchive ar(serverf);
                serverConfig.redisServerPort=6379;
                serverConfig.redisServerIp="127.0.0.1";
                serverConfig.client_vec.push_back("127.0.0.1");
                serverConfig.packageSize=10;
                ar(serverConfig);
            }
        }
    }

    if(bDistributed_matching && !bIsDistributedServer)
    {
        std::ifstream configFile(sConfigFile);
        if(configFile.is_open())
        {
            cereal::JSONInputArchive ar(configFile);
            ar(clientConfig);
        }
        else
        {
            std::ofstream serverf("serverconfig.json");
            if(serverf.is_open())
            {
                cereal::JSONOutputArchive ar(serverf);
                clientConfig.redisServerPort=6379;
                clientConfig.redisServerIp="127.0.0.1";
                clientConfig.serverIp="127.0.0.1";
                clientConfig.localIp="127.0.0.1";
                ar(clientConfig);
            }
        }
    }


    EPairMode ePairmode = (iMatchingVideoMode == -1 ) ? PAIR_EXHAUSTIVE : PAIR_CONTIGUOUS;

    if (sPredefinedPairList.length()) {
        ePairmode = PAIR_FROM_FILE;
        if (iMatchingVideoMode>0) {
            std::cerr << "\nIncompatible options: --videoModeMatching and --pairList" << std::endl;
            return EXIT_FAILURE;
        }
    }

    if ((sMatchesDirectory.empty() || !stlplus::is_folder(sMatchesDirectory)) && (bIsDistributedServer||!bDistributed_matching))  {
        std::cerr << "\nIt is an invalid output directory" << std::endl;
        return EXIT_FAILURE;
    }

    EGeometricModel eGeometricModelToCompute = FUNDAMENTAL_MATRIX;
    std::string sGeometricMatchesFilename = "";
    switch(sGeometricModel[0])
    {
        case 'f': case 'F':
            eGeometricModelToCompute = FUNDAMENTAL_MATRIX;
            sGeometricMatchesFilename = "matches.f.bin";
            break;
        case 'e': case 'E':
            eGeometricModelToCompute = ESSENTIAL_MATRIX;
            sGeometricMatchesFilename = "matches.e.bin";
            break;
        case 'h': case 'H':
            eGeometricModelToCompute = HOMOGRAPHY_MATRIX;
            sGeometricMatchesFilename = "matches.h.bin";
            break;
        default:
            std::cerr << "Unknown geometric model" << std::endl;
            return EXIT_FAILURE;
    }

    // -----------------------------
    // - Load SfM_Data Views & intrinsics data
    // a. Compute putative descriptor matches
    // b. Geometric filtering of putative matches
    // + Export some statistics
    // -----------------------------

    //---------------------------------------
    // Read SfM Scene (image view & intrinsics data)
    //---------------------------------------
    using namespace openMVG::features;
    SfM_Data sfm_data;
    PairWiseMatches map_PutativesMatches;
    std::unique_ptr<Matcher> collectionMatcher;
    std::vector<std::string> vec_fileNames;
    std::vector<std::pair<size_t, size_t> > vec_imagesSize;
    if(!bDistributed_matching || bIsDistributedServer)
    {
        if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
            std::cerr << std::endl
            << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
            return EXIT_FAILURE;
        }

        //---------------------------------------
        // Load SfM Scene regions
        //---------------------------------------
        // Init the regions_type from the image describer file (used for image regions extraction)

        const std::string sImage_describer = stlplus::create_filespec(sMatchesDirectory, "image_describer", "json");
        regions_type = Init_region_type_from_file(sImage_describer);
        if (!regions_type)
        {
            std::cerr << "Invalid: "
            << sImage_describer << " regions type file." << std::endl;
            return EXIT_FAILURE;
        }
        //---------------------------------------
        // a. Compute putative descriptor matches
        //    - Descriptor matching (according user method choice)
        //    - Keep correspondences only if NearestNeighbor ratio is ok
        //---------------------------------------

        // Load the corresponding view regions
        if (!regions_provider->load(sfm_data, sMatchesDirectory, regions_type)) {
            std::cerr << std::endl << "Invalid regions." << std::endl;
            return EXIT_FAILURE;
        }


        // Build some alias from SfM_Data Views data:
        // - List views as a vector of filenames & image sizes
        {
            vec_fileNames.reserve(sfm_data.GetViews().size());
            vec_imagesSize.reserve(sfm_data.GetViews().size());
            for (Views::const_iterator iter = sfm_data.GetViews().begin();
                 iter != sfm_data.GetViews().end();
                 ++iter)
            {
                const View * v = iter->second.get();
                vec_fileNames.push_back(stlplus::create_filespec(sfm_data.s_root_path,
                                                                 v->s_Img_path));
                vec_imagesSize.push_back( std::make_pair( v->ui_width, v->ui_height) );
            }
        }
        //Upload region type to redis

        redisConnnector.connect(serverConfig.redisServerIp,serverConfig.redisServerPort,[] (cpp_redis::redis_client&) {
            std::cout << "client disconnected (disconnection handler)" << std::endl;
        });
        {
            std::string serializedJson;
            std::stringstream ss;
            {
                cereal::JSONOutputArchive ar(ss);
                ar(regions_type);
            }
            serializedJson = ss.str();
            std::stringstream keystream;
            keystream << "redis_region_type";
            {
                redisConnnector.set(keystream.str(), serializedJson);
                redisConnnector.sync_commit();//Use Sync_commit for test
            }
        }
        {
            C_Progress_display my_progress_bar(regions_provider->regions_per_view.size(),
                                               std::cout, "\n- Regions Uploading -\n");
            std::vector<std::string> keys_vec;
//            keys_vec.resize(regions_provider->regions_per_view.size());
//#ifdef OPENMVG_USE_OPENMP
//    #pragma omp parallel
//#endif
            for (auto &kv:regions_provider->regions_per_view) {
//#ifdef OPENMVG_USE_OPENMP
//#pragma omp single nowait
//#endif
                int index = kv.first;
                std::string serializedJson;
                std::stringstream ss;
                {
                    cereal::JSONOutputArchive ar(ss);
                    ar(kv.second);
                }
                serializedJson = ss.str();
                std::stringstream keystream;
                keystream << "redis" << index;
                {
                    redisConnnector.set(keystream.str(), serializedJson);
                    redisConnnector.sync_commit();//Use Sync_commit for test
                }
                keys_vec.push_back(keystream.str());
                {
                    ++my_progress_bar;
                }

            }
            {
                std::string serializedJson;
                std::stringstream ss;
                {
                    cereal::JSONOutputArchive ar(ss);
                    ar(keys_vec);
                }
                serializedJson = ss.str();
                std::stringstream keystream;
                keystream << "redis_keys";
                {
                    redisConnnector.set(keystream.str(), serializedJson);
                    redisConnnector.sync_commit();//Use Sync_commit for test
                }
            }
            //Upload region provider to redis
        }

        std::cout << std::endl << " - PUTATIVE MATCHES - " << std::endl;

    }
    // If the matches already exists, reload them
    if
            (
            !bForce
            && (stlplus::file_exists(sMatchesDirectory + "/matches.putative.txt")
                || stlplus::file_exists(sMatchesDirectory + "/matches.putative.bin"))
            )
    {
        if(!bDistributed_matching || bIsDistributedServer)
        {
            if (!(Load(map_PutativesMatches, sMatchesDirectory + "/matches.putative.bin") ||
                  Load(map_PutativesMatches, sMatchesDirectory + "/matches.putative.txt")) )
            {
                std::cerr << "Cannot load input matches file";
                return EXIT_FAILURE;
            }
            std::cout << "\t PREVIOUS RESULTS LOADED;"
            << " #pair: " << map_PutativesMatches.size() << std::endl;
        }
    }
    else // Compute the putative matches
    {
        if(bDistributed_matching&&!bIsDistributedServer)
        {
            //Update region type from redis so that we can create matcher

            //It seems useless since we may never use AUTO matching on clients
        }
        std::cout << "Use: ";
        switch (ePairmode)
        {
            case PAIR_EXHAUSTIVE: std::cout << "exhaustive pairwise matching" << std::endl; break;
            case PAIR_CONTIGUOUS: std::cout << "sequence pairwise matching" << std::endl; break;
            case PAIR_FROM_FILE:  std::cout << "user defined pairwise matching" << std::endl; break;
        }

        // Allocate the right Matcher according the Matching requested method

        if (sNearestMatchingMethod == "AUTO")
        {
            if (regions_type.get()->IsScalar())
            {
                std::cout << "Using FAST_CASCADE_HASHING_L2 matcher" << std::endl;
                collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions_AllInMemory(fDistRatio));
            }
            else
            if (regions_type.get()->IsBinary())
            {
                std::cout << "Using BRUTE_FORCE_HAMMING matcher" << std::endl;
                collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, BRUTE_FORCE_HAMMING));
            }
        }
        else
        if (sNearestMatchingMethod == "BRUTEFORCEL2")
        {
            std::cout << "Using BRUTE_FORCE_L2 matcher" << std::endl;
            collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, BRUTE_FORCE_L2));
        }
        else
        if (sNearestMatchingMethod == "BRUTEFORCEHAMMING")
        {
            std::cout << "Using BRUTE_FORCE_HAMMING matcher" << std::endl;
            collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, BRUTE_FORCE_HAMMING));
        }
        else
        if (sNearestMatchingMethod == "ANNL2")
        {
            std::cout << "Using ANN_L2 matcher" << std::endl;
            collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, ANN_L2));
        }
        else
        if(sNearestMatchingMethod == "BRUTEFORCEGPU")
        {
            std::cout << "Using BRUTE_FORCE_GPU matcher" << std::endl;
            collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, BRUTE_FORCE_GPU));
        }
        else
        if (sNearestMatchingMethod == "CASCADEHASHINGL2")
        {
            std::cout << "Using CASCADE_HASHING_L2 matcher" << std::endl;
            collectionMatcher.reset(new Matcher_Regions_AllInMemory(fDistRatio, CASCADE_HASHING_L2));
        }
        else
        if (sNearestMatchingMethod == "FASTCASCADEHASHINGL2")
        {
            std::cout << "Using FAST_CASCADE_HASHING_L2 matcher" << std::endl;
            collectionMatcher.reset(new Cascade_Hashing_Matcher_Regions_AllInMemory(fDistRatio));
        }
        if (!collectionMatcher)
        {
            std::cerr << "Invalid Nearest Neighbor method: " << sNearestMatchingMethod << std::endl;
            return EXIT_FAILURE;
        }

        //Server Initialization
        Pair_Set pairs;
        {
            //Init Worker Queue

            // From matching mode compute the pair list that have to be matched:

            if(!bDistributed_matching || bIsDistributedServer) {
                switch (ePairmode) {
                    case PAIR_EXHAUSTIVE:
                        pairs = exhaustivePairs(sfm_data.GetViews().size());
                        break;
                    case PAIR_CONTIGUOUS:
                        pairs = contiguousWithOverlap(sfm_data.GetViews().size(), iMatchingVideoMode);
                        break;
                    case PAIR_FROM_FILE:
                        if (!loadPairs(sfm_data.GetViews().size(), sPredefinedPairList, pairs)) {
                            return EXIT_FAILURE;
                        };
                        break;
                }
            }

            if(bDistributed_matching && bIsDistributedServer) {
                pair_vec.resize(pairs.size());
                std::copy(pairs.begin(), pairs.end(), pair_vec.begin());
                std::cout << "Current process is working as server" << std::endl;
                initWorker(serverConfig);

                //Init socket

                zmqpp::socket_type type = zmqpp::socket_type::pull;
                recvsocket = new zmqpp::socket(context, type);
                std::stringstream bindUrl;
                bindUrl << "tcp://*:" << global_server_listen_port;
                recvsocket->bind(bindUrl.str());

                std::cout << "Start Dispatching jobs" << std::endl;
                std::cout << "Total jobs: " << pairs.size() << std::endl;

                //Send Sync message
                for(std::pair<std::string,std::string> kv: global_workerMap)
                {
                    //Send empty strat cmds to ask clients to quit
                    string workerId = kv.first;
                    SyncRegionCommand sync;
                    std::string commandJson;
                    std::ostringstream os;
                    {
                        cereal::JSONOutputArchive ar(os);
                        ar(sync);
                    }
                    commandJson = os.str();

                    global_worker_sockets[workerId]->send(commandJson, true);//No need to block

                }

                //Apprantlly we should block untill all the clients successfully synced the regions
                //So that we reach the correct result of matching only
                std::thread waitForSyncs(handleWaitForSync);
                waitForSyncs.join();
            }
        }

        //Client Initialization
        if(bDistributed_matching && !bIsDistributedServer)
        {

            redisConnnector.connect(clientConfig.redisServerIp,clientConfig.redisServerPort,[] (cpp_redis::redis_client&) {
                std::cout << "client disconnected (disconnection handler)" << std::endl;
            });
            //Create Matcher
            recvsocket = new zmqpp::socket(context,zmqpp::socket_type::pull);
            std::ostringstream recvUrl;
            recvUrl<<"tcp://*:"<<global_client_listen_port;
            recvsocket->bind(recvUrl.str());
            std::cout<<"Clinet started to listen on "<<global_client_listen_port<<std::endl;

            sendsocket = new zmqpp::socket(context,zmqpp::socket_type::push);
            std::ostringstream sendUrl;
            sendUrl<<"tcp://"<<clientConfig.serverIp<<":"<<global_server_listen_port;
            sendsocket->connect(sendUrl.str());

            //Wait for sync message so that we can make matches without considering the syncing cost
            bool bStartSync= false;
            while(!bStartSync)
            {
                std::string recvmessage;
                recvsocket->receive(recvmessage);//recv_block
                Command cmd;
                {
                    std::stringstream ss(recvmessage);
                    cereal::JSONInputArchive ar(ss);
                    ar(cmd);
                }
                if(cmd.command=="sync")
                {
                    bStartSync=true;
                }
            }
            map_PutativesMatches.clear();
            std::vector<std::string> keys_vec;
            //Get keys
            {

                std::string serializedJson;
                {
                    redisConnnector.get("redis_keys",[&](cpp_redis::reply& reply){
                        serializedJson = reply.as_string();
                    });
                    redisConnnector.sync_commit();//Use Sync_commit for test
                }
                std::stringstream ss(serializedJson);
                {
                    cereal::JSONInputArchive ar(ss);
                    ar(keys_vec);
                }

            }
            //Update region_type
            {
                std::string serializedJson;
                {
                    redisConnnector.get("redis_region_type",[&](cpp_redis::reply& reply){
                        serializedJson = reply.as_string();
                    });
                    redisConnnector.sync_commit();//Use Sync_commit for test
                }
                std::stringstream ss(serializedJson);
                {
                    cereal::JSONInputArchive ar(ss);
                    ar(regions_type);
                }
            }
            //Update region_provider

            for(auto key:keys_vec)
            {
                //
                std::string serializedJson;
                {
                    should_redis_replied= false;
                    redisConnnector.get(key,[&](cpp_redis::reply& reply){
                        serializedJson = reply.as_string();
                        should_redis_replied= true;
                    });
                    redisConnnector.sync_commit();//Use Sync_commit for test
                }
                while(!should_redis_replied)
                {
                    //wait for redis reply
                }
                std::unique_ptr<Regions> region(regions_type->EmptyClone());
                std::stringstream ss(serializedJson);
                {
                    cereal::JSONInputArchive ar(ss);
                    ar(region);
                }

                regions_provider->regions_per_view[std::stoi(key.substr(5))]=std::move(region);
            }
            {
                //Send an empty finishCommand to show that sync it down
                FinishTaskCommand finishTaskCommand;
                std::cout<<"Sync done:\t there are "<< regions_provider->regions_per_view.size()<<" regions" <<std::endl;
                finishTaskCommand.workerId = clientConfig.localIp;
                std::string sendmessage;
                std::ostringstream os;
                {
                    cereal::JSONOutputArchive o_archive(os);
                    o_archive(finishTaskCommand);
                }
                sendmessage = os.str();
                sendsocket->send(sendmessage);
            }
            //sfm_data seems useless, maybe we need to update it in the future
        }


        // Perform the matching
        system::Timer timer;
        {
            if (!bDistributed_matching)
            {

                // Photometric matching of putative pairs

                collectionMatcher->Match(sfm_data, regions_provider, pairs, map_PutativesMatches);

                //---------------------------------------
                //-- Export putative matches
                //---------------------------------------
                if (!Save(map_PutativesMatches, std::string(sMatchesDirectory + "/matches.putative.bin"))) {
                    std::cerr
                    << "Cannot save computed matches in: "
                    << std::string(sMatchesDirectory + "/matches.putative.bin");
                    return EXIT_FAILURE;
                }
            }
            else
            {
                if(bIsDistributedServer)
                {

                    std::thread recvThread = std::thread(handleServerRecvFinish);
                    //Schedule
                    while(global_next_task<pair_vec.size())
                    {
                        for(std::pair<std::string,std::string> kv: global_workerMap)
                        {
                            string workerId = kv.first;
                            if(isWorkerIdle(workerId))
                            {
                                int taskStart = global_next_task;
                                int taskEnd = global_next_task+serverConfig.packageSize>pair_vec.size()?pair_vec.size():global_next_task+serverConfig.packageSize;

                                //Prepare start matching
                                StartTaskCommand startTask;
                                Pair_Set& pairSet = startTask.tasks;
                                for(int i = taskStart;i<taskEnd;i++)
                                {
                                    pairSet.insert(pair_vec[i]);
                                }
                                //Send start matching
                                //
                                std::string commandJson;
                                std::stringstream os;
                                {//important: JSONOutputArchive will not end the streaming entil it is going to be destroyed
                                    cereal::JSONOutputArchive ar(os);
                                    ar(startTask);
                                }
                                commandJson = os.str();

                                global_worker_sockets[workerId]->send(commandJson);//No need to block

                                setWorkerBusy(workerId);
                                global_next_task = taskEnd;
                                std::cout<<"Job "<<taskStart<<"-"<<taskEnd-1<<"have been dispatched to worker "<<workerId<<std::endl;
                            }
                        }
                    }

                    recvThread.join();
                    for(std::pair<std::string,std::string> kv: global_workerMap)
                    {
                        //Send empty strat cmds to ask clients to quit
                        string workerId = kv.first;
                        StartTaskCommand startTask;
                        std::string commandJson;
                        std::ostringstream os;
                        {
                            cereal::JSONOutputArchive ar(os);
                            ar(startTask);
                        }
                        commandJson = os.str();

                        global_worker_sockets[workerId]->send(commandJson, true);//No need to block
                        std::stringstream disconnectUrl;
                        disconnectUrl<<"tcp://"<<workerId<<":"<<global_client_listen_port;
                        global_worker_sockets[workerId]->disconnect(disconnectUrl.str());

                        delete global_worker_sockets[workerId];

                    }

//                    recvsocket->unbind(bindUrl.str());
                    delete recvsocket;

                    map_PutativesMatches.insert(global_result_map.begin(),global_result_map.end());
                }
                else
                {
                    //Make it simple, Just do recv-match-send in a single loop
                    int numOfMessagesRecved = 0;
                    bool bQuit = false;
                    while(!bQuit)
                    {
                        std::string recvmessage;
                        recvsocket->receive(recvmessage);//recv_block
                        numOfMessagesRecved++;
                        std::cout<<"Clinet recved "<<numOfMessagesRecved<<" message from server"<<std::endl;


                        std::istringstream isMesg(recvmessage);

                        StartTaskCommand startCmd;
                        {
                            cereal::JSONInputArchive i_archive( isMesg );
                            i_archive( startCmd );
                        }
                        std::cout<<"StartCmd parsed successfully!"<<std::endl;
                        std::cout<<"\t there are "<<startCmd.tasks.size()<<" pairs need to be matched"<<std::endl;

                        if(startCmd.tasks.size()==0)
                        {
                            //Quit
                            std::ostringstream sendUrl;
                            sendUrl<<"tcp://"<<clientConfig.serverIp<<":"<<global_server_listen_port;
                            std::cout<<"Clinet quit"<<std::endl;
                            sendsocket->disconnect(sendUrl.str());
                            delete sendsocket;
                            delete recvsocket;
//                            recvsocket->unbind(recvUrl.str());
                            bQuit = true;
                            continue;
                        }

                        FinishTaskCommand finishTaskCommand;
                        collectionMatcher->Match(sfm_data, regions_provider, startCmd.tasks, finishTaskCommand.matches);

                        std::cout<<"Match done:\t there are "<<finishTaskCommand.matches.size()<<" match found"<<std::endl;
                        finishTaskCommand.workerId = clientConfig.localIp;

                        std::string sendmessage;
                        std::ostringstream os;
                        {
                            cereal::JSONOutputArchive o_archive(os);
                            o_archive(finishTaskCommand);
                        }
                        sendmessage = os.str();
////                        Try deserialization
//                        std::stringstream is(sendmessage);
//                        cereal::JSONInputArchive i_archive(is);
//                        FinishTaskCommand deseriealizedObj;
//                        i_archive(deseriealizedObj);
                        sendsocket->send(sendmessage);//send_block
//                        sleep(100);
                    }
                }
            }
        }
        std::cout << "Task (Regions Matching) done in (s): " << timer.elapsed() << std::endl;
    }

    if(bDistributed_matching&&!bIsDistributedServer)
    {
        //ask clients to quit
        return 0;
    }
    //-- export putative matches Adjacency matrix
    PairWiseMatchingToAdjacencyMatrixSVG(vec_fileNames.size(),
                                         map_PutativesMatches,
                                         stlplus::create_filespec(sMatchesDirectory, "PutativeAdjacencyMatrix", "svg"));
    //-- export view pair graph once putative graph matches have been computed
    {
        std::set<IndexT> set_ViewIds;
        std::transform(sfm_data.GetViews().begin(), sfm_data.GetViews().end(),
                       std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
        graph::indexedGraph putativeGraph(set_ViewIds, getPairs(map_PutativesMatches));
        graph::exportToGraphvizData(
                stlplus::create_filespec(sMatchesDirectory, "putative_matches"),
                putativeGraph.g);
    }

    //---------------------------------------
    // b. Geometric filtering of putative matches
    //    - AContrario Estimation of the desired geometric model
    //    - Use an upper bound for the a contrario estimated threshold
    //---------------------------------------

    std::unique_ptr<ImageCollectionGeometricFilter> filter_ptr(
            new ImageCollectionGeometricFilter(&sfm_data, regions_provider));

    if (filter_ptr)
    {
        system::Timer timer;
        std::cout << std::endl << " - Geometric filtering - " << std::endl;

        PairWiseMatches map_GeometricMatches;
        switch (eGeometricModelToCompute)
        {
            case HOMOGRAPHY_MATRIX:
            {
                const bool bGeometric_only_guided_matching = true;
                filter_ptr->Robust_model_estimation(GeometricFilter_HMatrix_AC(4.0, imax_iteration),
                                                    map_PutativesMatches, bGuided_matching,
                                                    bGeometric_only_guided_matching ? -1.0 : 0.6);
                map_GeometricMatches = filter_ptr->Get_geometric_matches();
            }
                break;
            case FUNDAMENTAL_MATRIX:
            {
                filter_ptr->Robust_model_estimation(GeometricFilter_FMatrix_AC(4.0, imax_iteration),
                                                    map_PutativesMatches, bGuided_matching);
                map_GeometricMatches = filter_ptr->Get_geometric_matches();
            }
                break;
            case ESSENTIAL_MATRIX:
            {
                filter_ptr->Robust_model_estimation(GeometricFilter_EMatrix_AC(4.0, imax_iteration),
                                                    map_PutativesMatches, bGuided_matching);
                map_GeometricMatches = filter_ptr->Get_geometric_matches();

                //-- Perform an additional check to remove pairs with poor overlap
                std::vector<PairWiseMatches::key_type> vec_toRemove;
                for (PairWiseMatches::const_iterator iterMap = map_GeometricMatches.begin();
                     iterMap != map_GeometricMatches.end(); ++iterMap)
                {
                    const size_t putativePhotometricCount = map_PutativesMatches.find(iterMap->first)->second.size();
                    const size_t putativeGeometricCount = iterMap->second.size();
                    const float ratio = putativeGeometricCount / (float)putativePhotometricCount;
                    if (putativeGeometricCount < 50 || ratio < .3f)  {
                        // the pair will be removed
                        vec_toRemove.push_back(iterMap->first);
                    }
                }
                //-- remove discarded pairs
                for (std::vector<PairWiseMatches::key_type>::const_iterator
                             iter =  vec_toRemove.begin(); iter != vec_toRemove.end(); ++iter)
                {
                    map_GeometricMatches.erase(*iter);
                }
            }
                break;
        }

        //---------------------------------------
        //-- Export geometric filtered matches
        //---------------------------------------
        if (!Save(map_GeometricMatches,
                  std::string(sMatchesDirectory + "/" + sGeometricMatchesFilename)))
        {
            std::cerr
            << "Cannot save computed matches in: "
            << std::string(sMatchesDirectory + "/" + sGeometricMatchesFilename);
            return EXIT_FAILURE;
        }

        std::cout << "Task done in (s): " << timer.elapsed() << std::endl;

        //-- export Adjacency matrix
        std::cout << "\n Export Adjacency Matrix of the pairwise's geometric matches"
        << std::endl;
        PairWiseMatchingToAdjacencyMatrixSVG(vec_fileNames.size(),
                                             map_GeometricMatches,
                                             stlplus::create_filespec(sMatchesDirectory, "GeometricAdjacencyMatrix", "svg"));

        //-- export view pair graph once geometric filter have been done
        {
            std::set<IndexT> set_ViewIds;
            std::transform(sfm_data.GetViews().begin(), sfm_data.GetViews().end(),
                           std::inserter(set_ViewIds, set_ViewIds.begin()), stl::RetrieveKey());
            graph::indexedGraph putativeGraph(set_ViewIds, getPairs(map_GeometricMatches));
            graph::exportToGraphvizData(
                    stlplus::create_filespec(sMatchesDirectory, "geometric_matches"),
                    putativeGraph.g);
        }
    }
    return EXIT_SUCCESS;
}
