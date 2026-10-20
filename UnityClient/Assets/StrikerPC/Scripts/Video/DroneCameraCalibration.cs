using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text.RegularExpressions;
using UnityEngine;

namespace StrikerPC.Video
{
    public sealed class DroneCameraCalibration : MonoBehaviour
    {
        private const string IntegratedRelativePath = "Maps/tello-camera-calibration.json";

        [SerializeField] private string relativePath = IntegratedRelativePath;
        [SerializeField] private bool loadOnAwake = true;
        [SerializeField] private int imageWidth = 960;
        [SerializeField] private int imageHeight = 720;
        [SerializeField] private float fx;
        [SerializeField] private float fy;
        [SerializeField] private float cx;
        [SerializeField] private float cy;
        [SerializeField] private float[] distortion = Array.Empty<float>();
        private float rmsReprojectionError;
        private bool loaded;

        public int ImageWidth => imageWidth;
        public int ImageHeight => imageHeight;
        public float Fx => fx;
        public float Fy => fy;
        public float Cx => cx;
        public float Cy => cy;
        public IReadOnlyList<float> Distortion => distortion;
        public float RmsReprojectionError => rmsReprojectionError;
        public bool IsValid => loaded
            && imageWidth > 0
            && imageHeight > 0
            && fx > 0f
            && fy > 0f;

        // 시작할 때 기존 calibration.json을 읽는다.
        private void Awake()
        {
            if (loadOnAwake)
            {
                Load();
            }
        }

        // OpenCV calibration.json의 내부 파라미터와 왜곡 계수를 읽는다.
        [ContextMenu("Calibration/Reload")]
        public bool Load()
        {
            ResetCalibration();
            relativePath = IntegratedRelativePath;
            string path = Path.Combine(
                Application.streamingAssetsPath,
                IntegratedRelativePath);
            try
            {
                return LoadJson(File.ReadAllText(path));
            }
            catch (Exception exception)
            {
                Debug.LogError($"[Drone Camera] calibration.json을 읽지 못했습니다: {exception.Message}", this);
                return false;
            }
        }

        // JSON 문자열 전체를 검증한 뒤에만 보정값을 한 번에 적용한다.
        private bool LoadJson(string json)
        {
            ResetCalibration();
            if (!ReadBoolean(json, "calibrated"))
            {
                Debug.LogWarning("[Drone Camera] 완료된 calibration.json이 아닙니다.", this);
                return false;
            }
            float[] matrix = ReadNumberArray(json, "camera_matrix");
            float[] coefficients = ReadNumberArray(json, "distortion_coefficients");
            float width = ReadScalar(json, "image_width", float.NaN);
            float height = ReadScalar(json, "image_height", float.NaN);
            float rms = ReadScalar(json, "rms_reprojection_error", float.NaN);
            if (matrix.Length != 9
                || coefficients.Length < 4
                || !AllFinite(matrix)
                || !AllFinite(coefficients)
                || !IsFinite(width)
                || !IsFinite(height)
                || width <= 0f
                || height <= 0f
                || !IsFinite(rms)
                || rms < 0f
                || rms > 1f
                || matrix[0] <= 0f
                || matrix[4] <= 0f)
            {
                Debug.LogWarning("[Drone Camera] calibration.json 보정값이 유효 범위를 벗어났습니다.", this);
                return false;
            }

            imageWidth = Mathf.RoundToInt(width);
            imageHeight = Mathf.RoundToInt(height);
            fx = matrix[0];
            cx = matrix[2];
            fy = matrix[4];
            cy = matrix[5];
            distortion = coefficients;
            rmsReprojectionError = rms;
            loaded = true;
            return true;
        }

        // 실패한 보정 로드가 이전 유효값을 남기지 않도록 초기화한다.
        private void ResetCalibration()
        {
            loaded = false;
            imageWidth = 0;
            imageHeight = 0;
            fx = 0f;
            fy = 0f;
            cx = 0f;
            cy = 0f;
            rmsReprojectionError = 0f;
            distortion = Array.Empty<float>();
        }

        // 배열의 모든 보정값이 유한한지 확인한다.
        private static bool AllFinite(IReadOnlyList<float> values)
        {
            for (int index = 0; index < values.Count; index++)
            {
                if (!IsFinite(values[index]))
                {
                    return false;
                }
            }
            return true;
        }

        // float 값이 NaN 또는 Infinity가 아닌지 확인한다.
        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        // 지정한 JSON bool 필드를 읽는다.
        private static bool ReadBoolean(string json, string key)
        {
            Match match = Regex.Match(
                json,
                $"\"{Regex.Escape(key)}\"\\s*:\\s*(true|false)",
                RegexOptions.IgnoreCase);
            return match.Success
                && bool.TryParse(match.Groups[1].Value, out bool value)
                && value;
        }

        // 지정한 JSON 배열 블록의 모든 숫자를 읽는다.
        private static float[] ReadNumberArray(string json, string key)
        {
            int keyIndex = json.IndexOf($"\"{key}\"", StringComparison.Ordinal);
            if (keyIndex < 0)
            {
                return Array.Empty<float>();
            }

            int start = json.IndexOf('[', keyIndex);
            if (start < 0)
            {
                return Array.Empty<float>();
            }

            int depth = 0;
            int end = -1;
            for (int index = start; index < json.Length; index++)
            {
                if (json[index] == '[')
                {
                    depth++;
                }
                else if (json[index] == ']' && --depth == 0)
                {
                    end = index;
                    break;
                }
            }

            if (end < start)
            {
                return Array.Empty<float>();
            }

            MatchCollection matches = Regex.Matches(
                json.Substring(start, end - start + 1),
                @"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?");
            List<float> values = new List<float>(matches.Count);
            foreach (Match match in matches)
            {
                if (float.TryParse(match.Value, NumberStyles.Float, CultureInfo.InvariantCulture, out float value))
                {
                    values.Add(value);
                }
            }
            return values.ToArray();
        }

        // 지정한 JSON 숫자 필드를 읽는다.
        private static float ReadScalar(string json, string key, float fallback)
        {
            Match match = Regex.Match(
                json,
                $"\"{Regex.Escape(key)}\"\\s*:\\s*([-+]?(?:\\d+\\.?\\d*|\\.\\d+)(?:[eE][-+]?\\d+)?)");
            return match.Success
                && float.TryParse(match.Groups[1].Value, NumberStyles.Float, CultureInfo.InvariantCulture, out float value)
                ? value
                : fallback;
        }
    }
}
