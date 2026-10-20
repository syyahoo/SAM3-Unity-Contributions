using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.UI;

namespace StrikerPC.XR
{
    public sealed class SubtitleHistoryView : MonoBehaviour
    {
        [SerializeField] private Text historyText;
        [SerializeField] private ScrollRect scrollRect;
        [SerializeField, Min(10)] private int maximumLines = 200;

        private readonly Queue<string> lines = new Queue<string>();
        private bool connectionWaiting;

        // LiveKit 미연결 상태를 자막 창에 표시한다.
        public void SetConnectionWaiting(bool waiting)
        {
            connectionWaiting = waiting;
            Render();
        }

        // 새 자막을 추가하고 마지막 줄로 스크롤한다.
        public void Append(string subtitle)
        {
            if (string.IsNullOrWhiteSpace(subtitle))
            {
                return;
            }

            lines.Enqueue(subtitle);
            while (lines.Count > maximumLines)
            {
                lines.Dequeue();
            }

            connectionWaiting = false;
            Render();

            if (scrollRect != null)
            {
                Canvas.ForceUpdateCanvases();
                scrollRect.verticalNormalizedPosition = 0f;
            }
        }

        // 표시 중인 자막 기록을 지운다.
        public void Clear()
        {
            lines.Clear();
            Render();
        }

        // 연결 상태와 누적 자막을 현재 Text에 반영한다.
        private void Render()
        {
            if (historyText == null)
            {
                return;
            }

            if (connectionWaiting)
            {
                historyText.text = "연결 대기중";
                return;
            }

            StringBuilder builder = new StringBuilder();
            foreach (string line in lines)
            {
                if (builder.Length > 0)
                {
                    builder.AppendLine();
                    builder.AppendLine();
                }
                builder.Append(line);
            }

            historyText.text = builder.ToString();
        }
    }
}
