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
                    JObject body = JsonConvert.DeserializeObject<JObject>(@event.EventBody.ToString().Trim());

                    _logger.LogWarning("Digital Twin Update");
                    _logger.LogInformation(body.ToString());

                    string device_id = "esp32s3-1";
                    int? desired_direction = null;
                    int? desired_mode = null;
                    double? desired_duty_cycle = null;
                    double? desired_velocity = null;
                    var device_twin = await registry_manager.GetTwinAsync(device_id);

                    JArray patches = (JArray)body["patch"];

                    bool update = false;
                    string path = "";

                    foreach (JObject patch in patches)
                    {
                        path = patch["path"].Value<string>();
                        switch (path)
                        {
                            case "/desired_mode":
                                desired_mode = patch["value"].Value<int>();
                                device_twin.Properties.Desired["desired_mode"] = desired_mode;
                                _logger.LogWarning("Desired Mode: {desired_mode}", desired_mode);
                                update = true;
                                break;
                            case "/desired_direction":
                                desired_direction = patch["value"].Value<int>();
                                device_twin.Properties.Desired["desired_direction"] = desired_direction;
                                _logger.LogWarning("Desired Direction: {desired_direction}", desired_direction);
                                update = true;
                                break;
                            case "/desired_duty_cycle":
                                desired_duty_cycle = patch["value"].Value<double>();
                                device_twin.Properties.Desired["desired_duty_cycle"] = desired_duty_cycle;
                                _logger.LogWarning("Desired Duty Cycle: {desired_duty_cycle}", desired_duty_cycle);
                                update = true;
                                break;
                            case "/desired_velocity":
                                desired_velocity = patch["value"].Value<double>();
                                device_twin.Properties.Desired["desired_velocity"] = desired_velocity;
                                _logger.LogWarning("Desired Velocity: {desired_velocity}", desired_velocity);
                                update = true;
                                break;
                            default:
                                break;
                        }
                    }
                    if (update)
                        await registry_manager.UpdateTwinAsync(device_twin.DeviceId, device_twin, device_twin.ETag);

                    // Append patch document for writtable digital twin properties
                    //desired_mode = device_twin.Properties.Desired["desired_mode"];
                    //desired_direction = device_twin.Properties.Desired["desired_direction"];
                    //desired_duty_cycle = device_twin.Properties.Desired["desired_duty_cycle"];
                    //desired_velocity = device_twin.Properties.Desired["desired_velocity"];

                    //digital_twin_patch.AppendReplace("/desired_mode", desired_mode);
                    //digital_twin_patch.AppendReplace("/desired_direction", desired_direction);
                    //digital_twin_patch.AppendReplace("/desired_duty_cycle", desired_duty_cycle);
                    //digital_twin_patch.AppendReplace("/desired_velocity", desired_velocity);
                    //await client.UpdateDigitalTwinAsync((string)device_id, digital_twin_patch);

                    //else if (type_dt_update)
                    //{
                    //    string device_id = body["id"].Value<string>();
                    //    string path = body["key"].Value<string>();
                    //    switch (path)
                    //    {
                    //        case "/desired_direction":
                    //            desired_direction = body["value"].Value<int>();
                    //            break;
                    //        case "/desired_mode":
                    //            desired_mode = body["value"].Value<int>();
                    //            break;
                    //        case "/desired_duty_cycle":
                    //            desired_duty_cycle = body["value"].Value<double>();
                    //            break;
                    //        case "/desired_velocity":
                    //            desired_velocity = body["value"].Value<double>();
                    //            break;
                    //        default:
                    //            break;
                    //    }
                    //    _logger.LogWarning("Desired Direction: {desired_direction}", desired_direction);
                    //    _logger.LogWarning("Desired Mode: {desired_mode}", desired_mode);
                    //    _logger.LogWarning("Desired Duty Cycle: {desired_duty_cycle}", desired_duty_cycle);
                    //    _logger.LogWarning("Desired Velocity: {desired_velocity}", desired_velocity);
                    //}
                }
            }
            catch (Exception ex)
            {
                _logger.LogError($"Error: {ex.Message}");
            }
        }
    }
}
