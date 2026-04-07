
// MFC_ITCP_UseDlg.cpp: 实现文件
//

#include "pch.h"
#include "framework.h"
#include "MFC_ITCP_Use.h"
#include "MFC_ITCP_UseDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CMFCITCPUseDlg 对话框



CMFCITCPUseDlg::CMFCITCPUseDlg(CWnd* pParent /*=nullptr*/)
	: CDialogEx(IDD_MFC_ITCP_USE_DIALOG, pParent)
	, m_sIP(_T("192.168.1.30"))
	, m_iPort(502)
	, m_d1(0)
	, m_d2(0)
	, m_d3(0)
	, m_d4(0)
	, m_d5(0)
	, m_d6(0)
	, m_dF1(0)
	, m_dF2(0)
	, m_dF3(0)
	, m_dF4(0)
	, m_dF5(0)
	, m_dF6(0)
	, m_sSavePath(_T("../data/itcp"))
	, m_strExtension (_T(".csv"))
	, m_FileCount(0)
	, m_iStep(20)
	, b_SaveData(false)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CMFCITCPUseDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT_IP, m_sIP);
	DDX_Text(pDX, IDC_EDIT_PORT, m_iPort);
	DDX_Text(pDX, IDC_EDIT3, m_d1);
	DDX_Text(pDX, IDC_EDIT4, m_d2);
	DDX_Text(pDX, IDC_EDIT5, m_d3);
	DDX_Text(pDX, IDC_EDIT6, m_d4);
	DDX_Text(pDX, IDC_EDIT7, m_d5);
	DDX_Text(pDX, IDC_EDIT8, m_d6);

	DDX_Text(pDX, IDC_EDIT9, m_dF1);
	DDX_Text(pDX, IDC_EDIT10, m_dF2);
	DDX_Text(pDX, IDC_EDIT11, m_dF3);
	DDX_Text(pDX, IDC_EDIT12, m_dF4);
	DDX_Text(pDX, IDC_EDIT13, m_dF5);
	DDX_Text(pDX, IDC_EDIT14, m_dF6);
	DDX_Text(pDX, IDC_EDIT15, m_sSavePath);
	DDX_Text(pDX, IDC_EDIT_STEP, m_iStep);
}

BEGIN_MESSAGE_MAP(CMFCITCPUseDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON_START, &CMFCITCPUseDlg::OnBnClickedButtonStart)
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON_REMOVE, &CMFCITCPUseDlg::OnBnClickedButtonRemove)
	ON_BN_CLICKED(IDC_BUTTON_REMOVE2, &CMFCITCPUseDlg::OnBnClickedButtonSave)
END_MESSAGE_MAP()


// CMFCITCPUseDlg 消息处理程序

BOOL CMFCITCPUseDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != nullptr)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CMFCITCPUseDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CMFCITCPUseDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CMFCITCPUseDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

#include <afxsock.h>
CString g_StrIP = _T("192.168.1.30");
int gPort = 502;

double V[6];

//tcpip程序代码
//ITCP采集卡线程
UINT Thread4ITCPDAQ(LPVOID pParam)
{
	//CProcessControl* P = (CProcessControl*)pParam;//固定格式

	WSADATA wsaData;
	int err = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (err != 0)
	{
		wprintf_s(L"WSAStartup failed with error: %d\n", err);
		return 0;
	}
	CString str_temp;
	CString m_strStatus;
	CSocket m_sockClient;
	CSocket m_sockServer;

	AfxSocketInit();

	if (!m_sockClient.Socket())
	{
		TRACE("ITCPDAQ开启失败");
		return false;
	}
	else
	{
		if (!m_sockClient.Connect(g_StrIP, gPort))
		{
			TRACE("ITCPDAQ连接失败");
			return false;
		}
	}

	char RevBuf[100] = { 0 };
	char SendBuf[12] = { 0x00, 0x03, 0x00, 0x00, 0x00, 0x06, 0x01, 0x04, 0x00, 0x40, 0x00, 0x08 };//查询指令

	INT16 Volt[6];
	char VoltTemp[12] = { 0 };
	int i = 0;
	double time_last = 0;
	while (1)
	{
		m_sockClient.Send(SendBuf, 12);
		int ret = m_sockClient.Receive(RevBuf, 50);
		if ((ret == SOCKET_ERROR) || (ret != 25))
		{
			TRACE("ITCPDAQ SOCKET_ERROR:%d\r\n", m_sockClient.GetLastError());
			continue;
		}

		for (int i = 0; i < 6; i++)//每组数据调换下高低位
		{
			VoltTemp[i * 2] = RevBuf[i * 2 + 9 + 1];
			VoltTemp[i * 2 + 1] = RevBuf[i * 2 + 9];
		}
		memcpy(Volt, VoltTemp, 12);
		V[0] = Volt[0] / 1000.0;
		V[1] = Volt[1] / 1000.0;
		V[2] = Volt[2] / 1000.0;
		V[3] = Volt[3] / 1000.0;
		V[4] = Volt[4] / 1000.0;
		V[5] = Volt[5] / 1000.0;

		double time = GetTickCount();
		i++;
		if (i == 100)
		{
			TRACE("time=%lf\r\n", time - time_last);
			time_last = time;
			i = 0;
		}


		//Sleep(4);
	}

	m_sockClient.Close();
	WSACleanup();
	TRACE("ITCPDAQ 线程退出\r\n");
	return 0;
}

void CMFCITCPUseDlg::OnBnClickedButtonStart()
{
	// TODO: 在此添加控件通知处理程序代码
	this->UpdateData(TRUE);
	g_StrIP = m_sIP;
	gPort = m_iPort;

	CWinThread* pWinThread4ITCPDAQ = AfxBeginThread(Thread4ITCPDAQ, this, 0, 0, THREAD_PRIORITY_NORMAL, NULL);

	SetTimer(1, m_iStep, NULL);
}


void CMFCITCPUseDlg::OnBnClickedButtonRemove()
{
	// TODO: 在此添加控件通知处理程序代码
	mdToZero[0] = 0.02382 * V[0] - 18.44574 * V[1] - 0.22239 * V[2] + 20.35616 * V[3] - 0.075587 * V[4] - 0.08534 * V[5];
	mdToZero[1] = 0.09196 * V[0] - 10.69917 * V[1] + 0.10793 * V[2] - 11.68687 * V[3] - 0.17341 * V[4] + 23.10986 * V[5];
	mdToZero[2] = -32.26721 * V[0] - 0.0741 * V[1] - 32.92776 * V[2] - 0.40465 * V[3] - 34.27623 * V[4] - 0.25102 * V[5];
	mdToZero[3] = -0.92114 * V[0] + 0.00242 * V[1] + 0.95334 * V[2] + 0.01167 * V[3] + 0.02876 * V[4] - 0.00295 * V[5];
	mdToZero[4] = -0.5652 * V[0] + 0.00209 * V[1] - 0.57045 * V[2] - 0.0208 * V[3] + 1.10965 * V[4] + 0.00627 * V[5];
	mdToZero[5] = -0.00204 * V[0] + 0.69008 * V[1] - 0.0084 * V[2] + 0.78457 * V[3] - 0.00537 * V[4] + 0.7375 * V[5];
}



void CMFCITCPUseDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 在此添加消息处理程序代码和/或调用默认值

	switch (nIDEvent) {
	case 1:
		m_d1 = MyRound(V[0]);
		m_d2 = MyRound(V[1]);
		m_d3 = MyRound(V[2]);
		m_d4 = MyRound(V[3]);
		m_d5 = MyRound(V[4]);
		m_d6 = MyRound(V[5]);

		m_dF1 = MyRound(0.02382 * V[0] - 18.44574 * V[1] - 0.22239 * V[2] + 20.35616 * V[3] - 0.075587 * V[4] - 0.08534 * V[5] - mdToZero[0]);
		m_dF2 = MyRound(0.09196 * V[0] - 10.69917 * V[1] + 0.10793 * V[2] - 11.68687 * V[3] - 0.17341 * V[4] + 23.10986 * V[5] - mdToZero[1]);
		m_dF3 = MyRound(-32.26721 * V[0] - 0.0741 * V[1] - 32.92776 * V[2] - 0.40465 * V[3] - 34.27623 * V[4] - 0.25102 * V[5] - mdToZero[2]);
		m_dF4 = MyRound(-0.92114 * V[0] + 0.00242 * V[1] + 0.95334 * V[2] + 0.01167 * V[3] + 0.02876 * V[4] - 0.00295 * V[5] - mdToZero[3]);
		m_dF5 = MyRound(-0.5652 * V[0] + 0.00209 * V[1] - 0.57045 * V[2] - 0.0208 * V[3] + 1.10965 * V[4] + 0.00627 * V[5] - mdToZero[4]);
		m_dF6 = MyRound(-0.00204 * V[0] + 0.69008 * V[1] - 0.0084 * V[2] + 0.78457 * V[3] - 0.00537 * V[4] + 0.7375 * V[5] - mdToZero[5]);

		if (b_SaveData) {

			CString strData;
			strData.Format(_T("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n"), m_d1, m_d2, m_d3, m_d4, m_d5, m_d6,
				m_dF1, m_dF2, m_dF3, m_dF4, m_dF5, m_dF6);

			m_CsvDataFile.WriteString(strData);
		}

		UpdateData(FALSE);

		break;
	}

	CDialogEx::OnTimer(nIDEvent);
}

double CMFCITCPUseDlg::MyRound(double dV) {
	return round(dV * 1000.0) / 1000.0;
}


void CMFCITCPUseDlg::OnBnClickedButtonSave()
{
	// TODO: 在此添加控件通知处理程序代码
	CString strTestPath;
	do {
		m_strFileName.Format(_T("%s_%d%s"), m_sSavePath, m_FileCount, m_strExtension);
		strTestPath = m_strFileName;

		// 检查文件是否存在
		CFile fileTest;
		if (!fileTest.Open(strTestPath, CFile::modeRead)) {
			// 文件不存在，可以使用这个文件名
			break;
		}
		fileTest.Close();

		// 文件存在，尝试下一个编号
		m_FileCount++;
	} while (true);

	// 更新保存路径显示
	if (!m_CsvDataFile.Open(m_strFileName, CFile::modeCreate | CFile::modeWrite)) {
		AfxMessageBox(_T("保存文件没有打开！"));
		return;
	}
	else {
		b_SaveData = true;
		if (m_CsvDataFile.GetLength() == 0) {
			CString strHeader = _T("V1,V2,V3,V4,V5,V6,Fx,Fy,Fz,Tx,Ty,Tz\n");
			m_CsvDataFile.WriteString(strHeader);
		}
	}
}
