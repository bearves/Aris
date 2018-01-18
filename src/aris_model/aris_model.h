#ifndef ARIS_MODEL_H
#define ARIS_MODEL_H

#include <functional>
#include <vector>
#include <aris_core.h>

namespace aris
{
    namespace model
    {
        /*
         * Class Model offers a space for users to 
         * store and share the controller's internal 
         * states across multiple gait functions.
         * It is needed be inherited for this usage.
         */
        class Model
        {
            public:
                Model();
                virtual ~Model();
                
                virtual int loadXml(const aris::core::XmlDocument& doc)
                {
                    return 0;
                }
        };

    }
}

#endif
