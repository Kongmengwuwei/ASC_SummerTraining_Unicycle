#include "menu.h"

#include <stddef.h>
#include <string.h>

static Menu_Item menu_item_pool[MENU_MAX_SIZE];
static uint8_t menu_item_count;

static void Create_Menu_Item(Menu_Item *father, Menu_Item *item,
                             const char *name, void *data, Menu_Kind kind)
{
    Menu_Item *brother;

    if ((father == NULL) || (item == NULL) || (name == NULL) ||
        (father->kind != MENU_Folder))
    {
        return;
    }

    memset(item, 0, sizeof(*item));
    item->name = name;
    item->data = data;
    item->kind = kind;
    item->Father = father;

    if (father->sons == 0U)
    {
        father->First_Son = item;
    }
    else
    {
        brother = father->First_Son;
        while (brother->Next_Brother != NULL)
        {
            brother = brother->Next_Brother;
        }
        brother->Next_Brother = item;
        item->Last_Brother = brother;
    }

    father->sons++;
    item->rank = father->sons;
}

void Menu_Tree_Reset(Menu_Item *root, const char *name)
{
    menu_item_count = 0U;
    memset(menu_item_pool, 0, sizeof(menu_item_pool));

    if (root != NULL)
    {
        memset(root, 0, sizeof(*root));
        root->name = name;
        root->kind = MENU_Folder;
    }
}

void All_Folder_Menu_Init(Menu_Item *menu)
{
    Menu_Item *first;
    Menu_Item *item;

    if ((menu == NULL) || (menu->First_Son == NULL))
    {
        return;
    }

    first = menu->First_Son;
    item = first;
    while (item != NULL)
    {
        if (item->kind == MENU_Folder)
        {
            All_Folder_Menu_Init(item);
        }
        if (item->Next_Brother == NULL)
        {
            break;
        }
        item = item->Next_Brother;
    }

    item->Next_Brother = first;
    first->Last_Brother = item;
}

void Create_Menu_Folder(Menu_Item *father, Menu_Item *item, const char *name)
{
    Create_Menu_Item(father, item, name, NULL, MENU_Folder);
}

void Create_Menu_File(Menu_Item *father, Menu_Item *item, const char *name,
                      void *data, Menu_Kind kind)
{
    Create_Menu_Item(father, item, name, data, kind);
}

Menu_Item *Create_Menu_Folder_dynamic(Menu_Item *father, const char *name)
{
    Menu_Item *item;

    if (menu_item_count >= MENU_MAX_SIZE)
    {
        return NULL;
    }

    item = &menu_item_pool[menu_item_count++];
    Create_Menu_Item(father, item, name, NULL, MENU_Folder);
    return item;
}

Menu_Item *Create_Menu_File_dynamic(Menu_Item *father, const char *name,
                                    void *data, Menu_Kind kind)
{
    Menu_Item *item;

    if ((data == NULL) || (menu_item_count >= MENU_MAX_SIZE))
    {
        return NULL;
    }

    item = &menu_item_pool[menu_item_count++];
    Create_Menu_Item(father, item, name, data, kind);
    return item;
}
