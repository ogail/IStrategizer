#ifndef RTSGRAPH_H
#define RTSGRAPH_H

#include <vector>
#include <boost/graph/adjacency_list.hpp>
#include "IDigraph.h"

namespace IStrategizer
{
    template<class TNodeValue>
    class RtsGraph
    {
    private:
        typedef typename boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, TNodeValue> BoostGraphType;
        typedef typename boost::graph_traits<BoostGraphType>::vertex_descriptor BoostVertex;

    public:
        BoostVertex AddNode(const _In_ TNodeValue& p_value)
        {
            BoostVertex vertex = boost::add_vertex(m_graph);
            m_graph[vertex] = p_value;
            return vertex;
        }
        void RemoveNode(const _In_ BoostVertex& p_vertex)
        {
            boost::remove_vertex(p_vertex, m_graph);
        }

        void AddEdge(const _In_ BoostVertex& p_source, const _In_ BoostVertex& p_destination)
        {
            boost::add_edge(p_source, p_destination, m_graph);
        }
        void RemoveEdge(const _In_ BoostVertex& p_source, const _In_ BoostVertex& p_destination)
        {
            boost::remove_edge(p_source, p_destination, m_graph);
        }

        TNodeValue GetNode(const _In_ BoostVertex& p_vertex)
        {
            return m_graph[p_vertex];
        }
        std::vector<TNodeValue> GetNodes()
        {
            std::vector<TNodeValue> nodes;

            for each(BoostVertex vertex in m_graph)
            {
                nodes.push_back(m_graph[vertex]);
            }

            return nodes;
        }
      
        size_t GetSize()
        {
            return boost::num_vertices(m_graph);
        }
        void Clear()
        {
            m_graph.clear();
        }
        
        std::vector<BoostVertex> GetChildren(const _In_ BoostVertex& p_parent)
        {
            std::vector<BoostVertex> children;

            for each(BoostVertex vertex in boost::adjacent_vertices(p_parent, m_graph))
            {
                children.push_back(vertex);
            }

            return children;
        }
        std::vector<BoostVertex> GetParents(const _In_ BoostVertex& p_child)
        {
            std::vector<BoostVertex> parents;

            for each(BoostVertex vertex in  boost::inv_adjacent_vertices(p_child, m_graph))
            {
                parents.push_back(vertex);
            }

            return parents;
        }
        
        std::vector<BoostVertex> GetRoots()
        {
            std::vector<BoostVertex> roots;

            for each(BoostVertex vertex in m_graph)
            {
                if(boost::in_degree(vertex, m_graph) == 0)
                {
                    roots.push_back(vertex);
                }
            }

            return roots;
        }
        std::vector<BoostVertex> GetLeaves()
        {
            std::vector<BoostVertex> leaves;

            for each(BoostVertex vertex in m_graph)
            {
                if(boost::out_degree(vertex, m_graph) == 0)
                {
                    leaves.push_back(vertex);
                }
            }

            return leaves;
        }

        bool AreConnected(const _In_ BoostVertex& p_source, const _In_ BoostVertex& p_destination)
        {
            return boost::lookup_edge(p_source, p_destination, m_graph).second;
        }

        BoostVertex GraphSubstitution(const _In_ std::vector<BoostVertex>& p_subgraphNodes, const _In_ TNodeValue& p_replacement)
        {
            BoostVertex vertex = boost::add_vertex(p_replacement, m_graph);
            GraphSubstitution(p_subgraphNodes, vertex);
            return vertex;
        }
        void GraphSubstitution(const _In_ std::vector<BoostVertex>& p_subgraphNodes, const _In_ BoostVertex& p_replacement)
        {
            for each(BoostVertex subGraphNode in p_subgraphNodes)
            {
                for each(BoostVertex parent in boost::inv_adjacent_vertices(subGraphNode, m_graph))
                {
                    boost::add_edge(parent, p_replacement, m_graph);
                }

                for each(BoostVertex child in boost::adjacent_vertices(subGraphNode, m_graph))
                {
                    boost::add_edge(p_replacement, child, m_graph);
                }

                boost::remove_vertex(subGraphNode, m_graph);
            }
        }

        bool IsSubGraphOf(RtsGraph<TNodeValue> p_otherGraph)
        {
            SubgraphCallback callback(boost::num_vertices(m_graph));
            boost::mcgregor_common_subgraphs(m_graph, p_otherGraph.m_graph, callback, true);
            return callback.m_matchFound;
        }
    private:
        BoostGraphType m_graph;
        
        struct SubgraphCallback
        {
            SubgraphCallback(size_t p_requiredSize)
            {
                m_matchFound = false;
                m_requiredSize = p_requiredSize;
            }

            template <typename MapOneToTwo, typename MapTwoToOne>
            bool operator()(MapOneToTwo p_mapOneToTwo, MapTwoToOne p_mapTwoToOne, size_t p_size)
            {
                return (m_matchFound = p_size >= m_requiredSize) == false;
            }
            
            bool m_matchFound;
            size_t m_requiredSize;
        };
    };
}

#endif