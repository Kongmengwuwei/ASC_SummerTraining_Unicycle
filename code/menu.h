#ifndef MENU_H_
#define MENU_H_

#include <stdbool.h>
#include <stdint.h>

#define MENU_MAX_SIZE (32U)

typedef enum
{
    MENU_Folder = 0,
    int32_Box,
    uint32_Box,
    int16_Box,
    uint16_Box,
    int8_Box,
    uint8_Box,
    float_Box,
    bool_Box
} Menu_Kind;

typedef struct Menu_Item
{
    const char *name;
    void *data;
    Menu_Kind kind;
    uint8_t sons;
    uint8_t rank;
    bool selected;
    struct Menu_Item *Father;
    struct Menu_Item *First_Son;
    struct Menu_Item *Next_Brother;
    struct Menu_Item *Last_Brother;
} Menu_Item;

void Menu_Tree_Reset(Menu_Item *root, const char *name);
void All_Folder_Menu_Init(Menu_Item *menu);
void Create_Menu_Folder(Menu_Item *father, Menu_Item *item, const char *name);
void Create_Menu_File(Menu_Item *father, Menu_Item *item, const char *name,
                      void *data, Menu_Kind kind);
Menu_Item *Create_Menu_Folder_dynamic(Menu_Item *father, const char *name);
Menu_Item *Create_Menu_File_dynamic(Menu_Item *father, const char *name,
                                    void *data, Menu_Kind kind);

#endif
