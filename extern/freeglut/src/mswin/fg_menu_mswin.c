/*
 * fg_menu_mswin.c
 *
 * The Windows-specific mouse cursor related stuff.
 *
 * Copyright (c) 2012 Stephen J. Baker. All Rights Reserved.
 * Written by John F. Fay, <fayjf@sourceforge.net>
 * Creation date: Sun Jan 22, 2012
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * PAWEL W. OLSZTA BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#define FREEGLUT_BUILDING_LIB
#include <GL/freeglut.h>
#include "../fg_internal.h"


extern void fgEnumMenus( FGCBMenuEnumerator enumCallback, SFG_Enumerator* enumerator );



GLvoid fgPlatformGetGameModeVMaxExtent( SFG_Window* window, int* x, int* y )
{
    *x = glutGet ( GLUT_SCREEN_WIDTH );
    *y = glutGet ( GLUT_SCREEN_HEIGHT );
}

static void fghcbIsActiveMenu(SFG_Menu *menu,
    SFG_Enumerator *enumerator)
{
    if (enumerator->found)
        return;

    /* Check the menu's active and the one we are searching for. */
    if (menu->IsActive && menu->Window->Window.Handle==(HWND)enumerator->data)
    {
        enumerator->found = GL_TRUE;
        enumerator->data  = (void*) menu;
        return;
    }
}

void fgPlatformCheckMenuDeactivate(HWND newFocusWnd)
{
    /* User/system switched application focus.
     * If we have an open menu, close it.
     * If the window that got focus is an active
     * menu window, don't do anything. This occurs
     * as it is sadly necessary to do an activating
     * ShowWindow() for the menu to pop up over the
     * gamemode window.
     * If the window that got focus is the gamemode
     * window, the menus pop under it. Bring them
     * back in view in this special case.
     */
    SFG_Menu* menu = NULL;
    SFG_Enumerator enumerator;

    if ( fgState.ActiveMenus )
    {
        /* see if there is an active menu whose window matches the one that got focus */
        enumerator.found = GL_FALSE;
        enumerator.data  = (void*) newFocusWnd;
        fgEnumMenus(fghcbIsActiveMenu, &enumerator);
        if (enumerator.found)
            menu = (SFG_Menu*) enumerator.data;

        if ( !menu )
        {
            /* window that got focus was not one of the active menus. That means we'll
             * close the active menu's unless the window that got focus was their parent */
            menu = fgGetActiveMenu();
            
            if (newFocusWnd != menu->ParentWindow->Window.Handle)
            {
                /* focus shifted to another window than the menu's parent, close menus */
                fgDeactivateMenu(menu->ParentWindow);
                return;
            }
        }
    }
};



/* -- PLATFORM-SPECIFIC INTERFACE FUNCTION -------------------------------------------------- */

int FGAPIENTRY __glutCreateMenuWithExit( void(* callback)( int ), void (__cdecl *exit_function)(int) )
{
  __glutExitFunc = exit_function;
  return glutCreateMenu( callback );
}

