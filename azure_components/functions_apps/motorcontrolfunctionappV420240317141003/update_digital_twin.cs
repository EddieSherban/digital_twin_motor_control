using Azure;
using Azure.DigitalTwins.Core;
using Azure.Identity;
using Azure.Messaging.EventHubs;
using Microsoft.Azure.Functions.Worker;
using Microsoft.Extensions.Logging;
using Newtonsoft.Json.Linq;

namespace motorcontrolfunctionappV420240317141003
{
    public class update_digital_twin
    {
        private readonly ILogger<update_digital_twin> _logger;

        public update_digital_twin(ILogger<update_digital_twin> logger)
        {
            _logger = logger;
        }

        private static readonly string ADT_SERVICE_URL = Environment.GetEnvironmentVariable("ADT_SERVICE_URL");
        private static readonly string CLIENT_ID = Environment.GetEnvironmentVariable("CLIENT_ID");

        [Function(nameof(update_digital_twin))]
        public async Task Run([EventHubTrigger("dtmc-event-hub", Connection = "TELEMETRY_EVENT_HUB")] EventData[] events)
        {
            Parallel.ForEach(events, async @event =>
            {
                try
                {
                    if (@event.SystemProperties.TryGetValue("iothub-connection-device-id", out var temp_device_id))
                    {
                        string telemetry_string = @event.EventBody.ToString();

                        _logger.LogWarning("Telemetry");
                        _logger.LogWarning(telemetry_string);

                        string device_id = (string)temp_device_id;
                        JObject telemetry_json = JObject.Parse(telemetry_string);
                        long[] timestamp_array = telemetry_json["timestamp"].ToObject<long[]>();
                        int[] direction_array = telemetry_json["direction"].ToObject<int[]>();
                        double[] duty_cycle_array = telemetry_json["duty_cycle"].ToObject<double[]>();
                        double[] velocity_array = telemetry_json["velocity"].ToObject<double[]>();
                        double[] position_array = telemetry_json["position"].ToObject<double[]>();
                        double[] current_array = telemetry_json["current"].ToObject<double[]>();

                        JsonPatchDocument digital_twin_patch = new JsonPatchDocument();
                        DateTime unix_epoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
                        DateTime timestamp;

                        var credentials = new ManagedIdentityCredential(CLIENT_ID, default);
                        var client = new DigitalTwinsClient(new Uri(ADT_SERVICE_URL), credentials);

                        int length = timestamp_array.Length;
                        length = Math.Min(length, direction_array.Length);
                        length = Math.Min(length, duty_cycle_array.Length);
                        length = Math.Min(length, velocity_array.Length);
                        length = Math.Min(length, position_array.Length);
                        length = Math.Min(length, current_array.Length);

                        Parallel.For(0, length - 1, i =>
                        {
                            digital_twin_patch = new JsonPatchDocument();
                            timestamp = unix_epoch.AddMilliseconds(timestamp_array[i]);

                            digital_twin_patch.AppendReplace("/duty_cycle", duty_cycle_array[i]);
                            digital_twin_patch.AppendReplace("/velocity", velocity_array[i]);
                            digital_twin_patch.AppendReplace("/position", position_array[i]);
                            digital_twin_patch.AppendReplace("/current", current_array[i]);

                            digital_twin_patch.AppendReplace("/$metadata/duty_cycle/sourceTime", timestamp);
                            digital_twin_patch.AppendReplace("/$metadata/velocity/sourceTime", timestamp);
                            digital_twin_patch.AppendReplace("/$metadata/position/sourceTime", timestamp);
                            digital_twin_patch.AppendReplace("/$metadata/current/sourceTime", timestamp);

                            client.UpdateDigitalTwinAsync(device_id, digital_twin_patch);
                        });
                    }
                }
                catch (Exception ex)
                {
                    _logger.LogError($"Error: {ex.Message}");
                }
            });
        }
    }
}
