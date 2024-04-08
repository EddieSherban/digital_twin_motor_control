using Azure.Messaging.EventHubs;
using Microsoft.Azure.Devices;
using Microsoft.Azure.Functions.Worker;
using Microsoft.Extensions.Logging;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace motorcontrolfunctionappV420240317141003
{
    public class update_device_twin
    {
        private readonly ILogger<update_device_twin> _logger;

        public update_device_twin(ILogger<update_device_twin> logger)
        {
            _logger = logger;
        }

        private static readonly string ADT_SERVICE_URL = Environment.GetEnvironmentVariable("ADT_SERVICE_URL");
        private static readonly string CLIENT_ID = Environment.GetEnvironmentVariable("CLIENT_ID");
        private static readonly string IOT_HUB_CONNECTION_STRING = Environment.GetEnvironmentVariable("IOT_HUB_CONNECTION_STRING");
        private static readonly RegistryManager registry_manager = RegistryManager.CreateFromConnectionString(IOT_HUB_CONNECTION_STRING);

        [Function(nameof(update_device_twin))]
        public async Task Run([EventHubTrigger("dtmc-event-hub", Connection = "DIGITAL_TWIN_EVENT_HUB")] EventData[] events)
        {
            try
            {
                foreach (EventData @event in events)
                {
                    string event_body = @event.EventBody.ToString();
                    bool digital_twin_update = false;

                    if (event_body.Contains("current"))
                        break;

                    _logger.LogWarning(event_body);

                    JObject body = JsonConvert.DeserializeObject<JObject>(event_body);

                    _logger.LogWarning("Digital Twin Update");
                    _logger.LogInformation(body.ToString());

                    string device_id = "esp32s3-1";
                    int? desired_direction = null;
                    int? desired_mode = null;
                    double? desired_duty_cycle = null;
                    double? desired_velocity = null;

                    bool update = false;
                    string path = "";

                    JArray patches = (JArray)body["patch"];
                    foreach (JObject patch in patches)
                    {
                        path = patch["path"].Value<string>();
                        switch (path)
                        {
                            case "/desired_mode":
                                desired_mode = patch["value"].Value<int>();
                                _logger.LogWarning("Desired Mode: {desired_mode}", desired_mode);
                                update = true;
                                break;
                            case "/desired_direction":
                                desired_direction = patch["value"].Value<int>();
                                _logger.LogWarning("Desired Direction: {desired_direction}", desired_direction);
                                update = true;
                                break;
                            case "/desired_duty_cycle":
                                desired_duty_cycle = patch["value"].Value<double>();
                                _logger.LogWarning("Desired Duty Cycle: {desired_duty_cycle}", desired_duty_cycle);
                                update = true;
                                break;
                            case "/desired_velocity":
                                desired_velocity = patch["value"].Value<double>();
                                _logger.LogWarning("Desired Velocity: {desired_velocity}", desired_velocity);
                                update = true;
                                break;
                            default:
                                break;
                        }
                    }
                    if (update)
                    {
                        var device_twin = await registry_manager.GetTwinAsync(device_id);

                        if (desired_mode != null)
                            device_twin.Properties.Desired["desired_mode"] = desired_mode;
                        if (desired_direction != null)
                            device_twin.Properties.Desired["desired_direction"] = desired_direction;
                        if (desired_duty_cycle != null)
                            device_twin.Properties.Desired["desired_duty_cycle"] = desired_duty_cycle;
                        if (desired_velocity != null)
                            device_twin.Properties.Desired["desired_velocity"] = desired_velocity;

                        await registry_manager.UpdateTwinAsync(device_twin.DeviceId, device_twin, device_twin.ETag);
                    }
                }
            }
            catch (Exception ex)
            {
                _logger.LogError($"Error: {ex.Message}");
            }
        }
    }
}
