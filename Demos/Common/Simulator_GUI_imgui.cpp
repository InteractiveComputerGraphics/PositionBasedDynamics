#include "Simulator_GUI_imgui.h"
#include "Demos/Visualization/MiniGL.h"
#include "DemoBase.h"
#include "imguiParameters.h"
#include "Utils/FileSystem.h"
#include "Simulation/Simulation.h"
#include "Simulation/TimeManager.h"
#include "LogWindow.h"

#include "imgui.h"
#include "imgui_internal.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl2.h"


using namespace PBD;
using namespace Utilities;

Simulator_GUI_imgui::Simulator_GUI_imgui(DemoBase *base)
{	
	m_base = base;
	m_currentScaleIndex = 0;
	m_vsync = false;
	m_iniFound = false;
	m_showLogWindow = true;
}

Simulator_GUI_imgui::~Simulator_GUI_imgui(void)
{	
	imguiParameters::cleanup();
	delete m_logWindow;
}

void Simulator_GUI_imgui::init()
{
	m_logWindow = new LogWindow();

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	m_context = ImGui::CreateContext();

	// Add .ini handle for UserData type
	ImGuiSettingsHandler ini_handler;
	ini_handler.TypeName = "PBDSimulator";
	ini_handler.TypeHash = ImHashStr("PBDSimulator");
	ini_handler.ReadOpenFn = readOpenIni;
	ini_handler.ReadLineFn = readIni;
	ini_handler.WriteAllFn = writeIni;
	ini_handler.ApplyAllFn = applySettings;
	ini_handler.UserData = this;
	m_context->SettingsHandlers.push_back(ini_handler);

	// load ini file before window is created
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;           // Enable Docking
	ImGui::LoadIniSettingsFromDisk(io.IniFilename);
}

void Simulator_GUI_imgui::initStyle()
{
	m_context->Style = ImGuiStyle();

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();

	ImGuiStyle* style = &ImGui::GetStyle();
	ImVec4* colors = style->Colors;
	colors[ImGuiCol_Text] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
	colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.1f, 0.1f, 1.0f);
	style->FrameBorderSize = 0.5f;
	style->FrameRounding = 3.0f;
	style->TabBorderSize = 1.0f;
	style->WindowRounding = 6.0f;
	style->ScaleAllSizes(1);
}

void* Simulator_GUI_imgui::readOpenIni(ImGuiContext* ctx, ImGuiSettingsHandler* handler, const char* name)
{
	Simulator_GUI_imgui* gui = (Simulator_GUI_imgui*)handler->UserData;
	return &gui->m_userSettings;
}

void Simulator_GUI_imgui::readIni(ImGuiContext* ctx, ImGuiSettingsHandler* handler, void* entry, const char* line)
{ 
	Simulator_GUI_imgui* gui = (Simulator_GUI_imgui*)handler->UserData;
	UserSettings* settings = (UserSettings*)entry;
	int x, y, w, h;
	int i;
	if (sscanf(line, "pos=%d,%d", &x, &y) == 2) { settings->win_x = x; settings->win_y = y; }
	else if (sscanf(line, "size=%d,%d", &w, &h) == 2) { settings->win_width = w; settings->win_height = h; }
	else if (sscanf(line, "scale=%d", &i) == 1) { settings->scaleIndex = i; }
	else if (sscanf(line, "maximized=%d", &i) == 1) { settings->maximized = (i != 0); }
	else if (sscanf(line, "vsync=%d", &i) == 1) { settings->vsync = (i != 0); }
	else if (sscanf(line, "show_log_window=%d", &i) == 1) { settings->show_log_window = (i != 0); }
	else if (sscanf(line, "log_filter=%d", &i) == 1) { settings->log_filter = i; }
}

void Simulator_GUI_imgui::writeIni(ImGuiContext* ctx, ImGuiSettingsHandler* handler, ImGuiTextBuffer* out_buf)
{ 
	Simulator_GUI_imgui* gui = (Simulator_GUI_imgui*)handler->UserData;
	out_buf->reserve(out_buf->size() + 200);
	out_buf->appendf("[%s][%s]\n", handler->TypeName, "Settings");
	out_buf->appendf("scale=%d\n", gui->m_currentScaleIndex);

	int x, y;
	MiniGL::getWindowPos(x, y);
	out_buf->appendf("pos=%d,%d\n", x, y);

	int w, h;
	MiniGL::getWindowSize(w, h);
	out_buf->appendf("size=%d,%d\n", w, h);
	out_buf->appendf("maximized=%d\n", MiniGL::getWindowMaximized());

	out_buf->appendf("vsync=%d\n", gui->m_vsync);
	out_buf->appendf("show_log_window=%d\n", gui->m_showLogWindow);
	out_buf->appendf("log_filter=%d\n", gui->m_logWindow->getSelectedFilter());
}

void Simulator_GUI_imgui::applySettings(ImGuiContext* ctx, ImGuiSettingsHandler* handler)
{
	Simulator_GUI_imgui* gui = (Simulator_GUI_imgui*)handler->UserData;
	UserSettings* settings = (UserSettings*) &gui->m_userSettings;
	gui->m_currentScaleIndex = settings->scaleIndex;
	gui->m_vsync = settings->vsync;
	gui->m_showLogWindow = settings->show_log_window;
	gui->m_iniFound = true;
	gui->m_logWindow->setSelectedFilter(settings->log_filter);
}

void Simulator_GUI_imgui::initImgui()
{
	MiniGL::addKeyboardFunc([](int key, int scancode, int action, int mods) -> bool { ImGui_ImplGlfw_KeyCallback(MiniGL::getWindow(), key, scancode, action, mods); return ImGui::GetIO().WantCaptureKeyboard; });
	MiniGL::addCharFunc([](int key, int action) -> bool { ImGui_ImplGlfw_CharCallback(MiniGL::getWindow(), key); return ImGui::GetIO().WantCaptureKeyboard; });
	MiniGL::addMousePressFunc([](int button, int action, int mods) -> bool { ImGui_ImplGlfw_MouseButtonCallback(MiniGL::getWindow(), button, action, mods); return ImGui::GetIO().WantCaptureMouse; });
	MiniGL::addMouseWheelFunc([](int pos, double xoffset, double yoffset) -> bool { ImGui_ImplGlfw_ScrollCallback(MiniGL::getWindow(), xoffset, yoffset); return ImGui::GetIO().WantCaptureMouse; });

	MiniGL::addKeyFunc('r', std::bind(&DemoBase::reset, m_base));
	MiniGL::addKeyFunc('w', Simulator_GUI_imgui::switchDrawMode);
	MiniGL::addKeyFunc(' ', std::bind(&Simulator_GUI_imgui::switchPause, this));
	MiniGL::setClientDestroyFunc(std::bind(&Simulator_GUI_imgui::destroy, this));

	// apply user settings from ini file 
	if (m_iniFound)
	{
		MiniGL::setWindowPos(m_userSettings.win_x, m_userSettings.win_y);
		MiniGL::setWindowSize(m_userSettings.win_width, m_userSettings.win_height);
	}

	ImGuiIO& io = ImGui::GetIO(); (void)io;
	
	std::string font = Utilities::FileSystem::normalizePath(m_base->getExePath() + "/resources/fonts/Roboto-Medium.ttf");
	std::string font2 = Utilities::FileSystem::normalizePath(m_base->getExePath() + "/resources/fonts/Cousine-Regular.ttf");

	m_scales.push_back(1.0f);
	m_scales.push_back(1.25f);
	m_scales.push_back(1.5f);
	m_scales.push_back(1.75f);
	m_scales.push_back(2.0f);

	for(int i=0; i < 5; i++)
		m_fonts.push_back(io.Fonts->AddFontFromFileTTF(font.c_str(), m_baseSize * m_scales[i]));
	for (int i = 0; i < 5; i++)
		m_fonts2.push_back(io.Fonts->AddFontFromFileTTF(font2.c_str(), m_baseSize * m_scales[i]));

	initStyle();

	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL(MiniGL::getWindow(), false);
	const char* glsl_version = "#version 330";
	ImGui_ImplOpenGL2_Init();
}

void Simulator_GUI_imgui::initImguiParameters()
{
	imguiParameters::imguiNumericParameter<Real>* timeParam = new imguiParameters::imguiNumericParameter<Real>();
	timeParam->description = "Current simulation time";
	timeParam->label = "Time";
	timeParam->readOnly = true;
	timeParam->getFct = []() -> Real { return TimeManager::getCurrent()->getTime(); };
	imguiParameters::addParam("General", "General", timeParam);

	imguiParameters::imguiNumericParameter<Real>* timeStepSizeParam = new imguiParameters::imguiNumericParameter<Real>();
	timeStepSizeParam->description = "Set time step size";
	timeStepSizeParam->label = "Time step size";
	timeStepSizeParam->minValue = static_cast<Real>(0.00001);
	timeStepSizeParam->maxValue = static_cast<Real>(0.1);
	timeStepSizeParam->getFct = []() -> Real { return TimeManager::getCurrent()->getTimeStepSize(); };
	timeStepSizeParam->setFct = [](Real v) { TimeManager::getCurrent()->setTimeStepSize(v); };
	imguiParameters::addParam("General", "General", timeStepSizeParam);

	imguiParameters::imguiBoolParameter* wireframeParam = new imguiParameters::imguiBoolParameter();
	wireframeParam->description = "Switch wireframe mode";
	wireframeParam->label = "Wireframe";
	wireframeParam->readOnly = false;
	wireframeParam->getFct = []() -> bool { return MiniGL::getDrawMode() == GL_LINE; };
	wireframeParam->setFct = [](bool v) {
		if (!v)
			MiniGL::setDrawMode(GL_FILL);
		else
			MiniGL::setDrawMode(GL_LINE);
	};
	imguiParameters::addParam("Visualization", "General", wireframeParam);
}

bool Simulator_GUI_imgui::alignedButton(const char* label, float alignment)
{
	ImGuiStyle& style = ImGui::GetStyle();

	const float size = ImGui::CalcTextSize(label).x + style.FramePadding.x * 2.0f;
	const float avail = ImGui::GetContentRegionAvail().x;

	const float offset = (avail - size) * alignment;
	if (offset > 0.0f)
		ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);

	return ImGui::Button(label);
}

void Simulator_GUI_imgui::createMenuBar()
{
	bool openpopup = false;

	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0.0f, 10.0f));

	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("Simulation"))
		{
			if (ImGui::MenuItem("Pause/run simulation", "Space"))
				switchPause();
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("GUI"))
		{
			if (ImGui::MenuItem("V-Sync", "", m_vsync))
			{
				m_vsync = !m_vsync;
				openpopup = true;
			}
			if (ImGui::MenuItem("Scale - 100%", "", m_currentScaleIndex == 0))
			{
				m_currentScaleIndex = 0;
				initStyle();
			}
			if (ImGui::MenuItem("Scale - 125%", "", m_currentScaleIndex == 1))
			{
				m_currentScaleIndex = 1;
				initStyle();
			}
			if (ImGui::MenuItem("Scale - 150%", "", m_currentScaleIndex == 2))
			{
				m_currentScaleIndex = 2;
				initStyle();
			}
			if (ImGui::MenuItem("Scale - 175%", "", m_currentScaleIndex == 3))
			{
				m_currentScaleIndex = 3;
				initStyle();
			}
			if (ImGui::MenuItem("Scale - 200%", "", m_currentScaleIndex == 4))
			{
				m_currentScaleIndex = 4;
				initStyle();
			}
			ImGui::Separator();
			if (ImGui::MenuItem("Show log window", "", m_showLogWindow))
			{
				m_showLogWindow = !m_showLogWindow;
			}
			ImGui::EndMenu();
		}
		ImGui::EndMainMenuBar();
	}

	ImGui::PopStyleVar(1);

	if (openpopup)
	{
		ImGui::OpenPopup("Info");
		openpopup = false;
	}
	bool open = true;
	ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5.0f, 5.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowTitleAlign, ImVec2(0.5f, 0.5f));
	ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(5.0f, 15.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
	if (ImGui::BeginPopupModal("Info", &open, ImGuiWindowFlags_AlwaysAutoResize))
	{
		ImGui::Text("To turn on/off the vertical sync, \nyou have to restart the simulator.");
		if (alignedButton("Close"))
			ImGui::CloseCurrentPopup();
		ImGui::EndPopup();
	}
	ImGui::PopStyleVar(4);
}

void Simulator_GUI_imgui::createSimulationParameterGUI()
{
	ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(390, 900), ImGuiCond_FirstUseEver);

	float alpha = 0.8f;
	if (ImGui::IsWindowDocked())
		alpha = 1.0f;

	ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.15f, 0.15f, 0.15f, alpha));

	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(5.0f, 10.0f));
	ImGui::Begin("Settings");

	ImGui::PushItemWidth(175 * m_scales[m_currentScaleIndex]);

	imguiParameters::createParameterGUI();

	ImGui::PopItemWidth();
	ImGui::End();
	ImGui::PopStyleVar(1);
	ImGui::PopStyleColor(1);

	if (m_showLogWindow)
		m_logWindow->drawWindow(m_fonts2[m_currentScaleIndex]);
}

void Simulator_GUI_imgui::initSimulationParameterGUI()
{
	imguiParameters::cleanup();

	Simulation *sim = Simulation::getCurrent();

	imguiParameters::createParameterObjectGUI(m_base);
	imguiParameters::createParameterObjectGUI(sim);
	if (sim->getModel() != nullptr)
		imguiParameters::createParameterObjectGUI(sim->getModel());
	if (sim->getTimeStep() != nullptr)
	{
		imguiParameters::createParameterObjectGUI(sim->getTimeStep());
		if (sim->getTimeStep()->getCollisionDetection() != nullptr)
			imguiParameters::createParameterObjectGUI(sim->getTimeStep()->getCollisionDetection());
	}
	initImguiParameters();
}

void Simulator_GUI_imgui::update()
{
	// Start the Dear ImGui frame
	ImGui_ImplGlfw_NewFrame();
	ImGui_ImplOpenGL2_NewFrame();
	ImGui::NewFrame();

	// init dock space
	static ImGuiDockNodeFlags dockspaceFlags = ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_NoDockingInCentralNode | ImGuiDockNodeFlags_AutoHideTabBar;
	ImGuiWindowFlags windowFlags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
	ImGuiViewport* viewport = ImGui::GetMainViewport();

	// add enough space for the menubar
	ImVec2 pos = viewport->Pos;
	ImVec2 size = viewport->Size;
	size.y -= m_baseSize * m_scales[m_currentScaleIndex];
	pos.y += m_baseSize * m_scales[m_currentScaleIndex];

	ImGui::SetNextWindowPos(pos);
	ImGui::SetNextWindowSize(size);
	ImGui::SetNextWindowViewport(viewport->ID);
	windowFlags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
	windowFlags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
	if (dockspaceFlags & ImGuiDockNodeFlags_PassthruCentralNode) 
		windowFlags |= ImGuiWindowFlags_NoBackground;

	ImGui::PushFont(m_fonts[m_currentScaleIndex]);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
	ImGui::Begin("DockSpace", nullptr, windowFlags);

	ImGuiID dockspaceID = ImGui::GetID("DockSpace");
	ImGui::DockSpace(dockspaceID, ImVec2(0.0f, 0.0f), dockspaceFlags);
	static int first = true;
	if (!m_iniFound && first)
	{
		first = false;
		ImGui::DockBuilderRemoveNode(dockspaceID);
		ImGui::DockBuilderAddNode(dockspaceID, dockspaceFlags | ImGuiDockNodeFlags_DockSpace);
		ImGui::DockBuilderSetNodeSize(dockspaceID, viewport->Size);

		auto dock_id_down = ImGui::DockBuilderSplitNode(dockspaceID, ImGuiDir_Down, 0.3f, nullptr, &dockspaceID);
		auto dock_id_left = ImGui::DockBuilderSplitNode(dockspaceID, ImGuiDir_Left, 0.3f, nullptr, &dockspaceID);
		ImGui::DockBuilderDockWindow("Settings", dock_id_left);
		ImGui::DockBuilderDockWindow("Log", dock_id_down);

		ImGui::DockBuilderFinish(dockspaceID);
	}

	ImGui::End();
	ImGui::PopStyleVar(3);


	createMenuBar();
	
	createSimulationParameterGUI();
	ImGui::PopFont();

	// Rendering
	ImGui::Render();
	ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
}

void Simulator_GUI_imgui::destroy()
{
	// Cleanup
	ImGui_ImplOpenGL2_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}

void Simulator_GUI_imgui::switchPause()
{
	m_base->setValue(DemoBase::PAUSE, !m_base->getValue<bool>(DemoBase::PAUSE));
}

void Simulator_GUI_imgui::switchDrawMode()
{
	if (MiniGL::getDrawMode() == GL_LINE)
		MiniGL::setDrawMode(GL_FILL);
	else
		MiniGL::setDrawMode(GL_LINE);
}

