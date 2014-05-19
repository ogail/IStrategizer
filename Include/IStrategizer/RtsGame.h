///> [Serializable]
#ifndef RTSGAME_H
#define RTSGAME_H

#include "EngineData.h"
#include "UserObject.h"
#include "SMap.h"
#include <vector>

namespace IStrategizer
{
    enum PlayerType;
    enum EntityClassType;
    enum ResearchType;
    class GamePlayer;
    class GameType;
    class GameEntity;
    class GameResearch;
    class GameRace;
    class WorldMap;

    typedef Serialization::SMap<EntityClassType, GameType*> EntiyTypesMap;
    typedef Serialization::SMap<ResearchType, GameResearch*> ResearchTypesMap;
    typedef Serialization::SMap<TID, GameRace*> RaceTypesMap;

    ///> class=RtsGame
    class RtsGame : public Serialization::UserObject
    {
        OBJECT_MEMBERS(1, &m_players);

    public:
        RtsGame() :
            m_pMap(nullptr),
            m_initialized(false),
            m_isOnline(true)
        {}

        virtual ~RtsGame();
        virtual void Init();
        virtual void DisplayMessage(const char* p_msg) = 0;

        void Players(std::vector<PlayerType>& p_playerIds) { m_players.Keys(p_playerIds); }
        void EntityTypes(std::vector<EntityClassType>& p_entityTypeIds) { sm_entityTypes.Keys(p_entityTypeIds); }
        void Researches(std::vector<ResearchType>& p_researchTypeIds) { sm_researchTypes.Keys(p_researchTypeIds); }

        const EntiyTypesMap& EntityTypes() const { return sm_entityTypes; }
        const ResearchTypesMap& ResearchTypes() const { return sm_researchTypes; }
        const RaceTypesMap& RaceTypes() const { return sm_raceTypes; }

        GamePlayer* GetPlayer(PlayerType p_id);
        GameType* GetEntityType(EntityClassType p_id);
        GameResearch* GetResearch(ResearchType p_id);
        GameRace* GetRace(TID raceID);
        GamePlayer* Self() { return GetPlayer(PLAYER_Self); }
        GamePlayer* Enemy() { return GetPlayer(PLAYER_Enemy); }
        WorldMap* Map() { _ASSERTE(m_initialized); return m_pMap; }
        bool IsOnline() const { return m_isOnline; }
        RtsGame* Snapshot() const;
        virtual size_t GetMaxTrainingQueueCount() const = 0;

    protected:
        void SetOffline();
        virtual void InitEntityTypes() = 0;
        virtual void InitResearchTypes() = 0;
        virtual void InitRaceTypes() = 0;
        virtual void InitPlayers() = 0;
        virtual void InitMap() = 0;
        virtual GameType* FetchEntityType(EntityClassType p_id) = 0;
        virtual GameResearch* FetchResearch(ResearchType p_id) = 0;
        virtual GamePlayer* FetchPlayer(PlayerType p_id) = 0;
        virtual int GetMaxForceSize() const = 0;
        virtual void Finalize();

        // Game types are shared across all RtsGame instances
        static EntiyTypesMap sm_entityTypes;
        static ResearchTypesMap sm_researchTypes;
        static RaceTypesMap sm_raceTypes;
        static bool sm_gameTypesInitialized;

        ///> type=map(pair(int,GamePlayer*))
        Serialization::SMap<PlayerType, GamePlayer*> m_players;
        ///> type=WorldMap*
        WorldMap* m_pMap;

        bool m_initialized;
        bool m_isOnline;
    };
}

extern IStrategizer::RtsGame* g_Game;

#endif // RTSGAME_H
