#include "TranslateTask.h"
#include "translate.hpp"

App::Translate *translate = new App::Translate();

void translateTask(void const * argument)
{
    translate->init();
    while (1)
    {
        translate->update();
        osDelay(1);
    }
}
