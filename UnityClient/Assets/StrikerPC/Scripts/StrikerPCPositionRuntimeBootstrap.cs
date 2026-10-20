using System;
using System.Reflection;
using StrikerPC.Control;
using StrikerPC.Map;
using StrikerPC.State;
using StrikerPC.Video;
using StrikerPC.XR;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace StrikerPC
{
    public static class StrikerPCPositionRuntimeBootstrap
    {
        // Scene 직렬화 변경 없이 통합 pose와 Quest 지도창을 연결한다.
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void Initialize()
        {
            ConfigureScene(SceneManager.GetActiveScene());
        }

        // 현재 Scene의 기존 오브젝트와 참조를 유지해 필요한 컴포넌트만 추가한다.
        private static void ConfigureScene(Scene scene)
        {
            if (!scene.IsValid())
            {
                return;
            }

            VenueMapLoader mapLoader = UnityEngine.Object.FindFirstObjectByType<VenueMapLoader>();
            MapPingUdpBridge pingBridge = UnityEngine.Object.FindFirstObjectByType<MapPingUdpBridge>();
            WorldPingManager pingManager = UnityEngine.Object.FindFirstObjectByType<WorldPingManager>();
            DronePoseHistoryProvider history =
                UnityEngine.Object.FindFirstObjectByType<DronePoseHistoryProvider>();
            GameObject leftWindow = GameObject.Find("LeftWindowAnchor");
            if (leftWindow != null
                && mapLoader != null
                && pingBridge != null
                && pingManager != null)
            {
                QuestMapPanel panel = leftWindow.GetComponent<QuestMapPanel>()
                    ?? leftWindow.AddComponent<QuestMapPanel>();
                panel.Configure(mapLoader, pingBridge, pingManager);
                panel.ConfigurePoseSource(history);
            }

            ConfigureDebugSubtitles();
            ConfigureMissionOverlay(pingBridge);
            ConfigureUnityHeartbeat();

            MonoBehaviour teamReceiver = FindTeamPoseReceiver();
            if (teamReceiver == null)
            {
                return;
            }

            PropertyInfo targetProperty = teamReceiver.GetType().GetProperty("PoseTarget");
            Transform poseTarget = targetProperty?.GetValue(teamReceiver) as Transform;
            teamReceiver.enabled = false;
            ContinuousDronePoseReceiver receiver =
                teamReceiver.GetComponent<ContinuousDronePoseReceiver>()
                ?? teamReceiver.gameObject.AddComponent<ContinuousDronePoseReceiver>();
            receiver.Configure(poseTarget);

            history?.ConfigurePoseSource(receiver);
            DroneMapTelemetryPublisher telemetry =
                UnityEngine.Object.FindFirstObjectByType<DroneMapTelemetryPublisher>();
            telemetry?.ConfigureTrackingSource(receiver);
            TelloTelemetryHud hud =
                UnityEngine.Object.FindFirstObjectByType<TelloTelemetryHud>(
                    FindObjectsInactive.Include);
            hud?.ConfigurePose(receiver);
            hud?.ConfigureAnchorOverlay(
                UnityEngine.Object.FindFirstObjectByType<DroneAnchorOverlay>(
                    FindObjectsInactive.Include));
        }

        // 기존 FPV 주석 기준에 임무 검출 오버레이를 런타임으로 연결한다.
        private static void ConfigureMissionOverlay(MapPingUdpBridge pingBridge)
        {
            PilotAnnotationOverlay annotation =
                UnityEngine.Object.FindFirstObjectByType<PilotAnnotationOverlay>(
                    FindObjectsInactive.Include);
            if (annotation == null)
            {
                return;
            }

            MissionVideoOverlay overlay = annotation.GetComponent<MissionVideoOverlay>()
                ?? annotation.gameObject.AddComponent<MissionVideoOverlay>();
            overlay.Configure(annotation);
            pingBridge?.ConfigureMissionOverlay(overlay);
        }

        // Scene 전환과 무관한 전용 host에 10Hz Unity heartbeat 송신기를 유지한다.
        private static void ConfigureUnityHeartbeat()
        {
            TelloControlOrchestrator control =
                UnityEngine.Object.FindFirstObjectByType<TelloControlOrchestrator>(
                    FindObjectsInactive.Include);
            UnityHeartbeatPublisher publisher =
                UnityEngine.Object.FindFirstObjectByType<UnityHeartbeatPublisher>(
                    FindObjectsInactive.Include);
            if (publisher == null)
            {
                GameObject host = new GameObject("StrikerPCUnityHeartbeat");
                UnityEngine.Object.DontDestroyOnLoad(host);
                publisher = host.AddComponent<UnityHeartbeatPublisher>();
            }
            publisher.Configure(control);
        }

        // Scene 직렬화 없이 localhost 디버그 자막을 기존 XR 자막창에 연결한다.
        private static void ConfigureDebugSubtitles()
        {
            SubtitleHistoryView[] views = UnityEngine.Object.FindObjectsByType<SubtitleHistoryView>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);
            if (views.Length == 0)
            {
                return;
            }

            GameObject host = GameObject.Find("StrikerPCRoot");
            if (host == null)
            {
                LiveKitAudioBridge audioBridge =
                    UnityEngine.Object.FindFirstObjectByType<LiveKitAudioBridge>(
                        FindObjectsInactive.Include);
                host = audioBridge != null ? audioBridge.gameObject : views[0].gameObject;
            }

            DebugSubtitleUdpReceiver receiver =
                host.GetComponent<DebugSubtitleUdpReceiver>()
                ?? host.AddComponent<DebugSubtitleUdpReceiver>();
            receiver.Configure(views[0]);
        }

        // Junction으로 연결된 팀 Receiver를 이름으로만 찾아 원본 참조를 유지한다.
        private static MonoBehaviour FindTeamPoseReceiver()
        {
            MonoBehaviour[] behaviours = UnityEngine.Object.FindObjectsByType<MonoBehaviour>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);
            foreach (MonoBehaviour behaviour in behaviours)
            {
                if (behaviour != null
                    && behaviour.GetType().FullName
                        == "TelloVisionTracking.DronePoseReceiver")
                {
                    return behaviour;
                }
            }
            return null;
        }
    }
}
