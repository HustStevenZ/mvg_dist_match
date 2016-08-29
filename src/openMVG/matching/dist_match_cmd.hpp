//
// Created by Sanqian on 16/8/19.
//

#ifndef OPENMVG_DIST_MATCH_CMD_HPP
#define OPENMVG_DIST_MATCH_CMD_HPP

#include "openMVG/types.hpp"

#include <cereal/cereal.hpp>
namespace openMVG {
    namespace matching {

        class Command{
        public:

            Command()= default;

            virtual ~Command()= default;
            template <class Archive>
            void  serialize( Archive & ar )
            {
                ar(cereal::make_nvp("commmand",command));
//                ar(command);
            }
        public:
            string command;

        };

        class FinishTaskCommand:public Command
        {

        public:
            FinishTaskCommand()
            {
//                command=Command_Type ::COMMAND_FINISHED;
                command = "finish";
                workerId = "127.0.0.1";
            }
            FinishTaskCommand(std::string worker)
            {
                command = "finish";
                workerId = worker;
            }

            template <class Archive>
                    void serialize(Archive& ar)
            {
                ar(cereal::make_nvp("command",command),cereal::make_nvp("workerId",workerId),cereal::make_nvp("matches",matches),cereal::make_nvp("taskId",taskId));
//                ar(command);
            }

        public:
            std::string workerId;
            PairWiseMatches matches;
            std::string taskId;
        };

        class SyncRegionCommand: public Command
        {
        public:
            SyncRegionCommand()
            {
                command="sync";
            }

        };

        class StartTaskCommand:public Command
        {
        public:
            StartTaskCommand()
            {
//                command=Command_Type::COMMAND_START;
                command="start";
            }

            StartTaskCommand(Pair_Set& taskSrc)
            {
                for(auto& kv:taskSrc)
                {
                    taskSrc.insert(kv);
                }
            }

            template <class Archive>
            void  serialize( Archive & ar )
            {
                ar(cereal::make_nvp("commmand",command),cereal::make_nvp("tasks",tasks),cereal::make_nvp("taskId",taskId));
//                ar(command);
            }

//            template<typename Archive>
//            static void load_and_construct(
//                    Archive &ar,
//                    cereal::construct<StartTaskCommand> &construct) {
//                std::string command;
//                Pair_Set tasks;
//
//                ar(cereal::make_nvp("command",command), cereal::make_nvp("tasks",tasks));
//                construct(tasks);
//            }
        public:
            Pair_Set tasks;
            std::string taskId;
        };
    }
}

#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::matching::FinishTaskCommand, "FinishTaskCommand");

CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::matching::StartTaskCommand, "StartTaskCommand");


CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::matching::SyncRegionCommand, "SyncRegionCommand");


#endif //OPENMVG_DIST_MATCH_CMD_HPP
