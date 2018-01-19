#ifndef ARIS_NODE_H
#define ARIS_NODE_H

#include <string>
#include <iostream>
#include <sstream>
#include <map>
#include <memory>
#include <algorithm>
#include <vector>
#include "aris_core.h"

namespace aris
{
    namespace server
    {
        class Node
        {
            public:
                Node* AddChildGroup(const char *Name);
                Node* AddChildUnique(const char *Name);
                Node* AddChildParam(const char *Name);
                Node* FindChild(const char *Name)
                {
                    auto result = std::find_if(children.begin(), children.end(), [Name](std::unique_ptr<Node> &node)
                            {
                            return (!std::strcmp(node->name.c_str(), Name));
                            });

                    if (result != children.end())
                    {
                        return result->get();
                    }
                    else
                    {
                        return nullptr;
                    }
                };

                bool IsTaken() { return isTaken; };
                void Take();
                void Reset()
                {
                    this->isTaken = false;
                    for (auto &child : children)
                    {
                        child->Reset();
                    }
                };

            public:
                Node(Node*father, const char *Name) :name(Name) { this->father = father; };
                virtual ~Node() {};

            private:
                std::string name;
                Node* father;
                std::vector<std::unique_ptr<Node> > children;

                bool isTaken{ false };

                friend void AddAllParams(const aris::core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames);
                friend void AddAllDefault(Node *pNode, std::map<std::string, std::string> &params);
        };
        class RootNode :public Node
        {
            public:
                RootNode(const char *Name) :Node(nullptr, Name) {};

            private:
                Node *pDefault;

                friend void AddAllParams(const aris::core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames);
                friend void AddAllDefault(Node *pNode, std::map<std::string, std::string> &params);
        };
        class GroupNode :public Node
        {
            public:
                GroupNode(Node*father, const char *Name) :Node(father, Name) {};
        };
        class UniqueNode :public Node
        {
            public:
                UniqueNode(Node*father, const char *Name) :Node(father, Name) {};

            private:
                Node *pDefault;

                friend void AddAllParams(const aris::core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames);
                friend void AddAllDefault(Node *pNode, std::map<std::string, std::string> &params);
        };
        class ParamNode :public Node
        {
            public:
                ParamNode(Node*father, const char *Name) :Node(father, Name) {};
            private:
                std::string type;
                std::string defaultValue;
                std::string minValue, maxValue;

                friend void AddAllParams(const aris::core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames);
                friend void AddAllDefault(Node *pNode, std::map<std::string, std::string> &params);
        };

        auto AddAllParams(const aris::core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames)->void;
        auto AddAllDefault(Node *pNode, std::map<std::string, std::string> &params)->void;

        struct CommandStruct
        {
            std::unique_ptr<RootNode> root;
            std::map<std::string, Node *> allParams{};
            std::map<char, std::string> shortNames{};

            CommandStruct(const std::string &name) :root(new RootNode(name.c_str())) {};
        };
    }
}
#endif
