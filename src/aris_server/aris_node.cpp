#include <aris_node.h>

namespace aris
{
    namespace server
    {
        Node* Node::AddChildGroup(const char *Name)
        {
            this->children.push_back(std::unique_ptr<Node>(new GroupNode(this, Name)));
            return children.back().get();
        };
        Node* Node::AddChildUnique(const char *Name)
        {
            this->children.push_back(std::unique_ptr<Node>(new UniqueNode(this, Name)));
            return children.back().get();
        }
        Node* Node::AddChildParam(const char *Name)
        {
            this->children.push_back(std::unique_ptr<Node>(new ParamNode(this, Name)));
            return children.back().get();
        }
        auto Node::Take()->void
        {
            if (dynamic_cast<RootNode*>(this))
            {
                if (this->isTaken)
                {
                    throw std::logic_error(std::string("Param ") + this->name + " has been inputed twice");
                }
                else
                {
                    this->isTaken = true;
                    return;
                }
            }
            else if (dynamic_cast<GroupNode*>(this))
            {
                if (this->isTaken)
                {
                    return;
                }
                else
                {
                    this->isTaken = true;
                    father->Take();
                }
            }
            else
            {
                if (this->isTaken)
                {
                    throw std::logic_error(std::string("Param ") + this->name + " has been inputed twice");
                }
                else
                {
                    this->isTaken = true;
                    father->Take();
                }
            }
        }

        auto AddAllParams(const aris::core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames)->void
        {
            //add all children//
            for (auto pChild = pEle->FirstChildElement(); pChild != nullptr; pChild = pChild->NextSiblingElement())
            {
                //check if children already has this value//
                if (pNode->FindChild(pChild->name()))
                {
                    throw std::runtime_error(std::string("XML file has error: node \"") + pChild->name() + "\" already exist");
                }

                //set all children//
                if (pChild->Attribute("type", "group"))
                {
                    AddAllParams(pChild, pNode->AddChildGroup(pChild->name()), allParams, shortNames);
                }
                else if (pChild->Attribute("type", "unique"))
                {
                    AddAllParams(pChild, pNode->AddChildUnique(pChild->name()), allParams, shortNames);
                }
                else
                {
                    //now the pChild is a param_node//
                    Node * insertNode;

                    if (allParams.find(std::string(pChild->name())) != allParams.end())
                    {
                        throw std::runtime_error(std::string("XML file has error: node \"") + pChild->name() + "\" already exist");
                    }
                    else
                    {
                        insertNode = pNode->AddChildParam(pChild->name());
                        allParams.insert(std::pair<std::string, Node *>(std::string(pChild->name()), insertNode));
                    }

                    /*set abbreviation*/
                    if (pChild->Attribute("abbreviation"))
                    {
                        if (shortNames.find(*pChild->Attribute("abbreviation")) != shortNames.end())
                        {
                            throw std::runtime_error(std::string("XML file has error: abbreviation \"") + pChild->Attribute("abbreviation") + "\" already exist");
                        }
                        else
                        {
                            char abbr = *pChild->Attribute("abbreviation");
                            shortNames.insert(std::pair<char, std::string>(abbr, std::string(pChild->name())));
                        }
                    }

                    /*set values*/
                    if (pChild->Attribute("type"))
                    {
                        dynamic_cast<ParamNode*>(insertNode)->type = std::string(pChild->Attribute("type"));
                    }
                    else
                    {
                        dynamic_cast<ParamNode*>(insertNode)->type = "";
                    }

                    if (pChild->Attribute("default"))
                    {
                        dynamic_cast<ParamNode*>(insertNode)->defaultValue = std::string(pChild->Attribute("default"));
                    }
                    else
                    {
                        dynamic_cast<ParamNode*>(insertNode)->defaultValue = "";
                    }

                    if (pChild->Attribute("maxValue"))
                    {
                        dynamic_cast<ParamNode*>(insertNode)->maxValue = std::string(pChild->Attribute("maxValue"));
                    }
                    else
                    {
                        dynamic_cast<ParamNode*>(insertNode)->maxValue = "";
                    }

                    if (pChild->Attribute("minValue"))
                    {
                        dynamic_cast<ParamNode*>(insertNode)->minValue = std::string(pChild->Attribute("minValue"));
                    }
                    else
                    {
                        dynamic_cast<ParamNode*>(insertNode)->minValue = "";
                    }
                }
            }

            /*set all values*/
            if (dynamic_cast<RootNode*>(pNode))
            {
                if (pEle->Attribute("default"))
                {
                    if (pNode->FindChild(pEle->Attribute("default")))
                    {
                        dynamic_cast<RootNode*>(pNode)->pDefault = pNode->FindChild(pEle->Attribute("default"));
                    }
                    else
                    {
                        throw std::logic_error(std::string("XML file has error: \"") + pNode->name + "\" can't find default param");
                    }
                }
                else
                {
                    dynamic_cast<RootNode*>(pNode)->pDefault = nullptr;
                }
            }

            if (dynamic_cast<UniqueNode*>(pNode))
            {
                if (pEle->Attribute("default"))
                {
                    if (pNode->FindChild(pEle->Attribute("default")))
                    {
                        dynamic_cast<UniqueNode*>(pNode)->pDefault = pNode->FindChild(pEle->Attribute("default"));
                    }
                    else
                    {
                        throw std::logic_error(std::string("XML file has error: \"") + pNode->name + "\" can't find default param");
                    }
                }
                else
                {
                    if (pNode->children.empty())
                    {
                        throw std::logic_error(std::string("XML file has error: unique node \"") + pNode->name + "\" must have more than 1 child");
                    }
                    else
                    {
                        dynamic_cast<UniqueNode*>(pNode)->pDefault = nullptr;
                    }
                }
            }
        }

        auto AddAllDefault(Node *pNode, std::map<std::string, std::string> &params)->void
        {
            if (pNode->isTaken)
            {
                if (dynamic_cast<RootNode*>(pNode))
                {
                    auto found = find_if(pNode->children.begin(), pNode->children.end(), [](std::unique_ptr<Node> &a)
                            {
                            return a->isTaken;
                            });

                    AddAllDefault(found->get(), params);
                }

                if (dynamic_cast<UniqueNode*>(pNode))
                {
                    auto found = find_if(pNode->children.begin(), pNode->children.end(), [](std::unique_ptr<Node> &a)
                            {
                            return a->isTaken;
                            });

                    AddAllDefault(found->get(), params);
                }

                if (dynamic_cast<GroupNode*>(pNode))
                {
                    for (auto &i : pNode->children)	AddAllDefault(i.get(), params);
                }

                if (dynamic_cast<ParamNode*>(pNode))
                {
                    if (params.at(pNode->name) == "")params.at(pNode->name) = dynamic_cast<ParamNode*>(pNode)->defaultValue;

                    return;
                }
            }
            else
            {
                if (dynamic_cast<RootNode*>(pNode))
                {
                    if (!pNode->children.empty())
                    {
                        if ((dynamic_cast<RootNode*>(pNode)->pDefault))
                        {
                            AddAllDefault(dynamic_cast<RootNode*>(pNode)->pDefault, params);
                        }
                        else
                        {
                            throw std::logic_error(std::string("cmd \"") + pNode->name + "\" has no default param");
                        }
                    }

                    pNode->isTaken = true;
                }

                if (dynamic_cast<UniqueNode*>(pNode))
                {
                    if (!pNode->children.empty())
                    {
                        if (dynamic_cast<UniqueNode*>(pNode)->pDefault)
                        {
                            AddAllDefault(dynamic_cast<UniqueNode*>(pNode)->pDefault, params);
                        }
                        else
                        {
                            throw std::logic_error(std::string("param \"") + pNode->name + "\" has no default sub-param");
                        }
                    }

                    pNode->isTaken = true;
                }

                if (dynamic_cast<GroupNode*>(pNode))
                {
                    for (auto &i : pNode->children)
                    {
                        AddAllDefault(i.get(), params);
                    }


                    pNode->isTaken = true;
                }

                if (dynamic_cast<ParamNode*>(pNode))
                {
                    params.insert(make_pair(pNode->name, dynamic_cast<ParamNode*>(pNode)->defaultValue));
                    pNode->isTaken = true;
                }
            }
        }

    }
}

