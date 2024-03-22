using Azure;
using Azure.DigitalTwins.Core;
using Azure.Identity;
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
        public async Task Run([EventHubTrigger("dtmc-event-hub", Connection = "EVENT_KEY_HUB")] EventData[] events)
        {
            var credentials = new ManagedIdentityCredential(CLIENT_ID, default);
            var client = new DigitalTwinsClient(new Uri(ADT_SERVICE_URL), credentials);

            try
            {
                foreach (EventData @event in events)
                {
                    bool type_telemetry = false;
                    bool type_patch = false;
                    bool type_dt_update = false;

                    JObject body = JsonConvert.DeserializeObject<JObject>(@event.EventBody.ToString());

                    if (body["action"] != null)
                    {
                        type_dt_update = true;
                        _logger.LogWarning("\n\nDigital Twin Update");
                    }
                    else if (body["patch"] != null)
                    {
                        type_patch = true;
                        _logger.LogWarning("\n\nPatch");
                    }
                    else
                    {
                        type_telemetry = true;
                        _logger.LogWarning("\n\nTelemetry");
                    }

                    _logger.LogInformation(body.ToString());

                    int desired_direction = 0;
                    int desired_mode = 0;
                    double desired_duty_cycle = 0;
                    double desired_velocity = 0;

                    if (type_telemetry)
                    {

                        @event.SystemProperties.TryGetValue("iothub-connection-device-id", out var device_id);
                        double duty_cycle = body["duty_cycle"].Value<double>();
                        double velocity = body["velocity"].Value<double>();
                        double position = body["position"].Value<double>();
                        double current = body["current"].Value<double>();
                        _logger.LogInformation("Duty Cycle: {duty_cycle}", duty_cycle);
                        _logger.LogInformation("Velocity: {velocity}", velocity);
                        _logger.LogInformation("Position: {position}", position);
                        _logger.LogInformation("Current: {current}", current);

                        // Create patch document for digital twin
                        JsonPatchDocument digital_twin_patch = new JsonPatchDocument();

                        // Update digital twin asynchronously with latest telemetry 
                        digital_twin_patch.AppendReplace("/duty_cycle", duty_cycle);
                        digital_twin_patch.AppendReplace("/velocity", velocity);
                        digital_twin_patch.AppendReplace("/position", position);
                        digital_twin_patch.AppendReplace("/current", current);
                        //_logger.LogInformation(digital_twin_patch.ToString());
                        await client.UpdateDigitalTwinAsync((string)device_id, digital_twin_patch);
                    }
                    else if (type_patch)
                    {
                        JArray patches = (JArray)body["patch"];

                        var device_twin = await registry_manager.GetTwinAsync("esp32s3-1");
                        bool update = false;
                        string path = string.Empty;

                        foreach (JObject patch in patches)
                        {
                            path = patch["path"].Value<string>();
                            switch (path)
                            {
                                case "/desired_direction":
                                    desired_direction = patch["value"].Value<int>();
                                    device_twin.Properties.Desired["desired_direction"] = desired_direction;
                                    update = true;
                                    break;
                                case "/desired_mode":
                                    desired_mode = patch["value"].Value<int>();
                                    device_twin.Properties.Desired["desired_mode"] = desired_mode;
                                    update = true;
                                    break;
                                case "/desired_duty_cycle":
                                    desired_duty_cycle = patch["value"].Value<double>();
                                    device_twin.Properties.Desired["desired_duty_cycle"] = desired_duty_cycle;
                                    update = true;
                                    break;
                                case "/desired_velocity":
                                    desired_velocity = patch["value"].Value<double>();
                                    device_twin.Properties.Desired["desired_velocity"] = desired_velocity;
                                    update = true;
                                    break;
                                default:
                                    break;
                            }
                            if (update)
                                await registry_manager.UpdateTwinAsync(device_twin.DeviceId, device_twin, device_twin.ETag);
                            update = false;
                        }

                        _logger.LogWarning("Desired Direction: {desired_direction}", desired_direction);
                        _logger.LogWarning("Desired Mode: {desired_mode}", desired_mode);
                        _logger.LogWarning("Desired Duty Cycle: {desired_duty_cycle}", desired_duty_cycle);
                        _logger.LogWarning("Desired Velocity: {desired_velocity}", desired_velocity);
                    }
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
                    await Task.Delay(5);
                }
            }
            catch (Exception ex)
            {
                _logger.LogError($"Error: {ex.Message}");
            }
            await Task.Delay(5);

        }
    }
}
