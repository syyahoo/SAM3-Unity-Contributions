# SAM3 Unity Contributions

3인 팀 프로젝트 SAM3에서 오성영이 담당한 Unity XR 클라이언트와 Python Gateway 코드 발췌본입니다.

[프로젝트 설명과 시연 영상](https://ohsungyoung-game.netlify.app/#sam3)

![SAM3](https://ohsungyoung-game.netlify.app/assets/sam3-poster.jpg)

## 본인 담당 범위

- 기간: 2026.07.13~2026.08.31 / Team VOXEL 3인 / 본인 기여도 40%
- Unity 기반 XR UI, 실제 Tello 드론과 XR 장비의 연결 및 비행 안정 기능
- Unity와 Python Gateway 사이의 영상·명령·텔레메트리 연동
- Tello·Quest 3 실기기에서 통합 동작 확인
- Codex를 구현 보조 도구로 활용하고 기능·동작 조건 구체화 및 실행 결과 검증

팀은 가상융합기술 아카데미 제5기 융합프로젝트 1위로 부총리 겸 과학기술정보통신부 장관상을 수상했습니다. 이 수상은 팀 성과이며, 본인 담당 범위와 구분합니다.

## 먼저 볼 코드

| 기능 | 코드 |
| --- | --- |
| XR 조종 입력 | [XrMode2ControlInput.cs](UnityClient/Assets/StrikerPC/Scripts/Control/XrMode2ControlInput.cs) |
| 조종 상태와 연결 흐름 | [TelloControlOrchestrator.cs](UnityClient/Assets/StrikerPC/Scripts/Control/TelloControlOrchestrator.cs) |
| 드론 상태 수신 | [TelloStateReceiver.cs](UnityClient/Assets/StrikerPC/Scripts/State/TelloStateReceiver.cs) |
| 조종 화면 상태 표시 | [TelloTelemetryHud.cs](UnityClient/Assets/StrikerPC/Scripts/State/TelloTelemetryHud.cs) |
| XR UI 배치 | [XrHudLayout.cs](UnityClient/Assets/StrikerPC/Scripts/XR/XrHudLayout.cs) |
| Python 측 조종 인터페이스 | [xr_control_adapter.py](Gateway/xr_control_adapter.py), [control_protocol.py](Gateway/control_protocol.py) |

입력은 Unity 조종 코드에서 Gateway로 전달되고, 드론의 상태·영상은 각 수신 경로를 통해 Unity 화면에 반영됩니다. 인터페이스와 안전·상태 처리는 별도 코드로 구성되어 있습니다.

## 코드 선정 기준

사용자가 GitHub 작성자 계정 `syyahoo`를 본인 코드 판단 기준으로 확인했습니다. 검토한 원본 기본 브랜치의 296개 커밋은 모두 해당 계정으로 연결되어 있었습니다. 이 중 `UnityClient/Assets/StrikerPC/Scripts`와 `Gateway`의 소스만 추렸습니다.

다른 팀 영역인 외부 upstream, Vision 원본, WebViewer, 지도 데이터, 모델, 생성물, 배포 환경 설정, Scene·Prefab·프로젝트 설정, 과거 Git 이력은 포함하지 않았습니다. 작성자 정보는 공개 범위 선정 근거이며 팀 프로젝트 전체의 권리를 주장하는 의미가 아닙니다.

## 환경과 검증 범위

원본 환경은 Unity 6, C#, OpenXR, Python입니다. 코드에는 XR Interaction Toolkit, Input System, LiveKit 등 원본 패키지와 외부 시스템에 의존하는 부분이 있습니다. 원본 실행 구성과 Inspector 연결이 생략된 코드 발췌본으로, 이 저장소만으로 전체 드론 운용 시스템이 실행되지는 않습니다.

공개 대상 소스의 인증정보 패턴과 파일 출처를 확인했으며 Python 파일의 구문을 검사했습니다. 이번 공개본에 대한 Unity 컴파일·기기 재시험은 수행하지 않았습니다. 파일별 원본 SHA와 공개 변경 사항은 [PUBLICATION_MANIFEST.json](PUBLICATION_MANIFEST.json)에 기록했습니다.

## 이용 범위

코드 열람과 개인 담당 기능의 기술 평가를 위한 자료입니다. 팀 전체 코드에 대한 재사용 권한이나 별도 오픈소스 라이선스를 부여하지 않습니다. 외부 시스템과 라이브러리는 각 원 저작자의 권리를 따릅니다.
