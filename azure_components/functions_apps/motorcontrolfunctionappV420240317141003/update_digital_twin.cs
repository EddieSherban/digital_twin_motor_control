using Azure;
using Azure.DigitalTwins.Core;
using Azure.Identity;
using Azure.Messaging.EventHubs;
using Microsoft.Azure.Functions.Worker;
using Microsoft.Extensions.Logging;
using Newtonsoft.Json;
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
            var credentials = new ManagedIdentityCredential(CLIENT_ID, default);
            var client = new DigitalTwinsClient(new Uri(ADT_SERVICE_URL), credentials);

            try
            {
                foreach (EventData @event in events)
                {
                    JObject body = JsonConvert.DeserializeObject<JObject>(@event.EventBody.ToString());

                    if (@event.SystemProperties.TryGetValue("iothub-connection-device-id", out var temp_device_id))
                    {
                        _logger.LogWarning("Telemetry");
                        _logger.LogInformation(body.ToString());

                        JsonPatchDocument digital_twin_patch = new JsonPatchDocument();
                        string device_id = (string)temp_device_id;
                        double duty_cycle = body["duty_cycle"].Value<double>();
                        double velocity = body["velocity"].Value<double>();
                        double position = body["position"].Value<double>();
                        double current = body["current"].Value<double>();

                        // Append patch document for non-writtable digital twin properties
                        digital_twin_patch.AppendReplace("/duty_cycle", duty_cycle);
                        digital_twin_patch.AppendReplace("/velocity", velocity);
                        digital_twin_patch.AppendReplace("/position", position);
                        digital_twin_patch.AppendReplace("/current", current);
                        await client.UpdateDigitalTwinAsync((string)device_id, digital_twin_patch);

                        _logger.LogWarning("Duty Cycle: {duty_cycle}", duty_cycle);
                        _logger.LogWarning("Velocity: {velocity}", velocity);
                        _logger.LogWarning("Position: {position}", position);
                        _logger.LogWarning("Current: {current}", current);
                    }
                    break;
                }
            }
            catch (Exception ex)
            {
                _logger.LogError($"Error: {ex.Message}");
            }
        }
    }
}
