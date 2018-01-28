using System;
using FormsJoystick.ViewModels;
using Xamarin.Forms;

namespace FormsJoystick.Views
{
    public class PageViewBase : ContentPage
    {
        /// <summary>
        /// Performs page clean-up.
        /// </summary>
        protected override void OnDisappearing()
        {
            base.OnDisappearing();

            var viewModel = BindingContext as ViewModelBase;

            // Inform the view model that it is disappearing so that it can remove event handlers
            // and perform any other clean-up required..
            viewModel?.OnDisappearing();
        }

        protected override void OnAppearing()
        {
            // Inform the view model that it is appearing
            var viewModel = BindingContext as ViewModelBase;

            // Inform the view model that it is appearing.
            viewModel?.OnAppearing();
        }
    }
}