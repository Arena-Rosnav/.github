
# Unity Simulator

The Unity simulator is an executable build from [this Unity Repository](https://github.com/TheZomb/arena-unity) and is given certain parameters from the CLI. 

## Add API Endpoint

You can easily add an endpoint (i.e. `http://localhost:8080/example`) by editing `RoutingManager.cs`.   
Here is one example of an Endpoint that could be added to `Start()`:  

```cs
server.EndpointCollection.RegisterEndpoint(HttpMethod.GET, "/example", (request) =>
{
    // Handle Request
    ...
});
```

or you could add a reference method

```cs
void Start()
{
    server.EndpointCollection.RegisterEndpoint(HttpMethod.GET, "/example", Handler);
}

private void Handler(RestRequest request) {
    // Handle Request
    ...
}
```

For more detailed documentation see [the official Documentation](https://markus-seidl.de/unity-restserver/).