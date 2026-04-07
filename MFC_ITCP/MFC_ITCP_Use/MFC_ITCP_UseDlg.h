
// MFC_ITCP_UseDlg.h: 头文件
//

#pragma once


// CMFCITCPUseDlg 对话框
class CMFCITCPUseDlg : public CDialogEx
{
// 构造
public:
	CMFCITCPUseDlg(CWnd* pParent = nullptr);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_MFC_ITCP_USE_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持
	CStdioFile m_CsvDataFile;
	double mdToZero[6];

	double MyRound(double dV);

// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedButtonStart();
	CString m_sIP;
	int m_iPort;
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	double m_d1;
	double m_d2;
	double m_d3;
	double m_d4;
	double m_d5;
	double m_d6;

	double m_dF1;
	double m_dF2;
	double m_dF3;
	double m_dF4;
	double m_dF5;
	double m_dF6;

	bool b_SaveData=false;
	CString m_strExtension;
	CString m_sSavePath;
	CString m_strFileName;
	int m_FileCount;
	int m_iStep;
	afx_msg void OnBnClickedButtonRemove();
	afx_msg void OnBnClickedButtonSave();
};
