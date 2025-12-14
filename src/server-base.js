// Snaply Server

modules.server_web = 'v1'
SnapExtensions.primitives.set('server_return()',
  function () {
    // Not supported in web environment
    return;
  })