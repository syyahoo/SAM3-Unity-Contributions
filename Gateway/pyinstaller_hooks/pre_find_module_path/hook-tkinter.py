# PyInstaller의 자동 Tcl 초기화가 실패해도 표준 라이브러리 tkinter 검색 경로를 유지한다.
def pre_find_module_path(hook_api):
    return None
