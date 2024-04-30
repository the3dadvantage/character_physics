# Character-Engine
[![paypal]<div id="paypal-button-container-P-8V2845643T4460310MYYRCZA"></div>
<script src="https://www.paypal.com/sdk/js?client-id=Abs4oYwLq8kXnP4dQ3hnaOp__43b4uE14aYC_QMbUFMXCb91qy3PTV7KevS_FuNQwYlPzOZXW3shwvG6&vault=true&intent=subscription" data-sdk-integration-source="button-factory"></script>
<script>
  paypal.Buttons({
      style: {
          shape: 'rect',
          color: 'gold',
          layout: 'vertical',
          label: 'subscribe'
      },
      createSubscription: function(data, actions) {
        return actions.subscription.create({
          /* Creates the subscription */
          plan_id: 'P-8V2845643T4460310MYYRCZA'
        });
      },
      onApprove: function(data, actions) {
        alert(data.subscriptionID); // You can add optional success message for the subscriber here
      }
  }).render('#paypal-button-container-P-8V2845643T4460310MYYRCZA'); // Renders the PayPal button
</script>
