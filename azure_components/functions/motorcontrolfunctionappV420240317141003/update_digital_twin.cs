// This is the default URL for triggering event grid function in the local environment.
// http://localhost:7071/admin/extensions/EventGridExtensionConfig?functionName={functionname} 

using Azure;
using Azure.DigitalTwins.Core;
using Azure.Identity;
using Azure.Messaging.EventGrid;
using Microsoft.Azure.Functions.Worker;
using Microsoft.Extensions.Logging;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System.Text;

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
        public async Task Run([EventGridTrigger] EventGridEvent eventGridEvent)
        {
            if (ADT_SERVICE_URL == null) _logger.LogError("Application setting \"ADT_SERVICE_URL\" not set");
            try
            {
                _logger.LogInformation(ADT_SERVICE_URL);

                var credentials = new ManagedIdentityCredential("063fad9a-cc4c-430c-8cc0-50d4b07b4c03", default);
                var client = new DigitalTwinsClient(new Uri(CLIENT_ID), credentials);

                _logger.LogInformation($"ADT service client connection created.");

                if (eventGridEvent != null && eventGridEvent.Data != null)
                {
                    _logger.LogInformation(eventGridEvent.Data.ToString());

                    // Converts message into JSON format
                    JObject message = (JObject)JsonConvert.DeserializeObject(eventGridEvent.Data.ToString());
                    byte[] body_bytes = Convert.FromBase64String((string)message["body"]);
                    string body_string = Encoding.UTF8.GetString(body_bytes);
                    dynamic body = JsonConvert.DeserializeObject(body_string);

                    // Get device id, and motor parameters telemetry
                    string deviceId = (string)message["systemProperties"]["iothub-connection-device-id"];
                    string model_id = (string)message["systemProperties"]["dt-dataschema"];
                    double duty_cycle = body.duty_cycle * 100.0;
                    double velocity = body.velocity;
                    double position = body.position;
                    double current = body.current;

                    // Display the motor parameters
                    _logger.LogInformation($"Device ID: {deviceId}, " +
                                            $"Duty Cycle (%): {duty_cycle}, " +
                                            $"Velocity (RPM): {velocity}, " +
                                            $"Position (Degrees): {position}, " +
                                            $"Current (mA): {current}");

                    // Create patch document for digital twin
                    JsonPatchDocument updateTwinData = new JsonPatchDocument();

                    // Update digital twin asynchronously with latest telemetry 
                    updateTwinData.AppendReplace("/duty_cycle", duty_cycle);
                    updateTwinData.AppendReplace("/velocity", velocity);
                    updateTwinData.AppendReplace("/position", position);
                    updateTwinData.AppendReplace("/current", current);
                    await client.UpdateDigitalTwinAsync(deviceId, updateTwinData);
                }
            }

            catch (Exception ex)
            {
                _logger.LogError($"Error in ingest function: {ex.Message}");
            }

        }
    }
}
